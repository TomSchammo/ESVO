#include <esvo_core/esvo_Tracking.h>
#include <esvo_core/tools/TicToc.h>
#include <esvo_core/tools/params_helper.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sys/stat.h>

//#define ESVO_CORE_TRACKING_DEBUG
//#define ESVO_CORE_TRACKING_DEBUG

namespace esvo_core
{
esvo_Tracking::esvo_Tracking()
  : rclcpp::Node("esvo_tracking"),
    calibInfoDir_(tools::param(this, "calibInfoDir", std::string(""))),
  camSysPtr_(new CameraSystem(calibInfoDir_, false)),
  rpConfigPtr_(new RegProblemConfig(
    tools::param(this, "patch_size_X", 25),
    tools::param(this, "patch_size_Y", 25),
    tools::param(this, "kernelSize", 15),
    tools::param(this, "LSnorm", std::string("l2")),
    tools::param(this, "huber_threshold", 10.0),
    tools::param(this, "invDepth_min_range", 0.0),
    tools::param(this, "invDepth_max_range", 0.0),
    tools::param(this, "MIN_NUM_EVENTS", 1000),
    tools::param(this, "MAX_REGISTRATION_POINTS", 500),
    tools::param(this, "BATCH_SIZE", 200),
    tools::param(this, "MAX_ITERATION", 10))),
  rpType_((RegProblemType)((size_t)tools::param(this, "RegProblemType", 0))),
  rpSolver_(camSysPtr_, rpConfigPtr_, rpType_, NUM_THREAD_TRACKING),
  ets_(IDLE)
{
  // offline data
  dvs_frame_id_        = tools::param(this, "dvs_frame_id", std::string("dvs"));
  world_frame_id_      = tools::param(this, "world_frame_id", std::string("world"));

  /**** online parameters ***/
  tracking_rate_hz_    = tools::param(this, "tracking_rate_hz", 100);
  TS_HISTORY_LENGTH_   = tools::param(this, "TS_HISTORY_LENGTH", 100);
  REF_HISTORY_LENGTH_  = tools::param(this, "REF_HISTORY_LENGTH", 5);
  bSaveTrajectory_     = tools::param(this, "SAVE_TRAJECTORY", false);
  bVisualizeTrajectory_ = tools::param(this, "VISUALIZE_TRAJECTORY", true);
  resultPath_          = tools::param(this, "PATH_TO_SAVE_TRAJECTORY", std::string());
  // system status - use shared topic for inter-node coordination (ROS2 parameters are node-local)
  system_status_pub_ = create_publisher<std_msgs::msg::String>("/ESVO_SYSTEM_STATUS", rclcpp::QoS(1).transient_local());
  system_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/ESVO_SYSTEM_STATUS", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      if (auto parsed = system_status_from_string(msg->data)) {
        ESVO_System_Status_.store(*parsed);
      } else {
        auto error_msg = std::format("Unknown /ESVO_SYSTEM_STATUS value: {}", msg->data.c_str());
        RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
        throw std::runtime_error(error_msg);
      }
    });

  // online data callbacks
  events_left_sub_ = this->create_subscription<dvs_msgs::msg::EventArray>(
    "events_left", rclcpp::QoS(10).best_effort(), std::bind(&esvo_Tracking::eventsCallback, this, std::placeholders::_1));
  time_offset_sub_ = this->create_subscription<builtin_interfaces::msg::Duration>(
    "/time_offset", rclcpp::QoS(10).best_effort(),
    [this](const builtin_interfaces::msg::Duration::SharedPtr msg) {
      this->time_offset_ = rclcpp::Duration(*msg);
    });
  // message_filters subscribers and synchronizer
  TS_left_sub_.subscribe(this, "time_surface_left", rmw_qos_profile_default);
  TS_right_sub_.subscribe(this, "time_surface_right", rmw_qos_profile_default);
  TS_sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy>>(ExactSyncPolicy(10), TS_left_sub_, TS_right_sub_);
  TS_sync_->registerCallback(std::bind(&esvo_Tracking::timeSurfaceCallback, this, std::placeholders::_1, std::placeholders::_2));
  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/esvo_tracking/pose_pub", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/esvo_tracking/trajectory", 1);
  // Subscribers
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud", 10, std::bind(&esvo_Tracking::refMapCallback, this, std::placeholders::_1));// local map in the ref view.
  stampedPose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "stamped_pose", 10, std::bind(&esvo_Tracking::stampedPoseCallback, this, std::placeholders::_1));// for accessing the pose of the ref view.

  /*** For Visualization and Test ***/
  reprojMap_pub_left_ = image_transport::create_publisher(this, "Reproj_Map_Left");
  rpSolver_.setRegPublisher(&reprojMap_pub_left_);

  /*** Tracker ***/
  T_world_cur_ = Eigen::Matrix<double,4,4>::Identity();
  std::thread TrackingThread(&esvo_Tracking::TrackingLoop, this);
  TrackingThread.detach();
}

esvo_Tracking::~esvo_Tracking()
{
}

void esvo_Tracking::TrackingLoop()
{
  rclcpp::Rate r(tracking_rate_hz_);
  while(rclcpp::ok())
  {
    // Keep Idling
    if(refPCMap_.size() < 1 || TS_history_.size() < 1)
    {
      r.sleep();
      continue;
    }
    // Reset - check system status (updated via shared topic subscription)
    if(ESVO_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == WORKING)// This is true when the system is reset from dynamic reconfigure
    {
      reset();
      r.sleep();
      continue;
    }
    if(ESVO_System_Status_.load() == SystemStatus::TERMINATE)
    {
      LOG(INFO) << "The tracking node is terminated manually...";
      break;
    }

    // Data Transfer (If mapping node had published refPC.)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if(ref_.t_.nanoseconds() < refPCMap_.rbegin()->first.nanoseconds())// new reference map arrived
        refDataTransferring();
      if(cur_.t_.nanoseconds() < TS_history_.rbegin()->first.nanoseconds())// new observation arrived
      {
        if(ref_.t_.nanoseconds() >= TS_history_.rbegin()->first.nanoseconds())
        {
          LOG(INFO) << "The time_surface observation should be obtained after the reference frame";
          exit(-1);
        }
        if(!curDataTransferring())
          continue;
      }
      else
        continue;
    }

    // create new regProblem
    TicToc tt;
    double t_resetRegProblem, t_solve, t_pub_result, t_pub_gt;
#ifdef  ESVO_CORE_TRACKING_DEBUG
    tt.tic();
#endif
    if(rpSolver_.resetRegProblem(&ref_, &cur_))
    {
#ifdef  ESVO_CORE_TRACKING_DEBUG
      t_resetRegProblem = tt.toc();
      tt.tic();
#endif
      if(ets_ == IDLE)
        ets_ = WORKING;
      if(ESVO_System_Status_.load() != SystemStatus::WORKING)
      {
        ESVO_System_Status_.store(SystemStatus::WORKING);
        auto status_msg = std_msgs::msg::String();
        status_msg.data = to_string(ESVO_System_Status_.load());
        system_status_pub_->publish(status_msg);
      }
      if(rpType_ == REG_NUMERICAL)
        rpSolver_.solve_numerical();
      if(rpType_ == REG_ANALYTICAL)
        rpSolver_.solve_analytical();
#ifdef ESVO_CORE_TRACKING_DEBUG
      t_solve = tt.toc();
      tt.tic();
#endif
      T_world_cur_ = cur_.tr_.getTransformationMatrix();
      publishPose(cur_.t_, cur_.tr_);
      if(bVisualizeTrajectory_)
        publishPath(cur_.t_, cur_.tr_);
#ifdef ESVO_CORE_TRACKING_DEBUG
      t_pub_result = tt.toc();
#endif

      // save result and gt if available.
      if(bSaveTrajectory_)
      {
        // save results to listPose and listPoseGt
        lTimestamp_.push_back(std::to_string(cur_.t_.seconds()));
        lPose_.push_back(cur_.tr_.getTransformationMatrix());
      }
    }
    else
    {
      ESVO_System_Status_.store(SystemStatus::INITIALIZATION);
      auto status_msg = std_msgs::msg::String();
      status_msg.data = to_string(ESVO_System_Status_.load());
      system_status_pub_->publish(status_msg);
      ets_ = IDLE;
//      LOG(INFO) << "Tracking thread is IDLE";
    }

#ifdef  ESVO_CORE_TRACKING_LOG
    double t_overall_count = 0;
    t_overall_count = t_resetRegProblem + t_solve + t_pub_result;
    LOG(INFO) << "\n";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "--------------------Tracking Computation Cost---------------";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "ResetRegProblem: " << t_resetRegProblem << " ms, (" << t_resetRegProblem / t_overall_count * 100 << "%).";
    LOG(INFO) << "Registration: " << t_solve << " ms, (" << t_solve / t_overall_count * 100 << "%).";
    LOG(INFO) << "pub result: " << t_pub_result << " ms, (" << t_pub_result / t_overall_count * 100 << "%).";
    LOG(INFO) << "Total Computation (" << rpSolver_.lmStatics_.nPoints_ << "): " << t_overall_count << " ms.";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "------------------------------------------------------------";
#endif
    r.sleep();
  }// while

  if(bSaveTrajectory_)
  {
    struct stat st;
    if( stat(resultPath_.c_str(), &st) == -1 )// there is no such dir, create one
    {
      LOG(INFO) << "There is no such directory: " << resultPath_;
      tools::_mkdir(resultPath_.c_str());
      LOG(INFO) << "The directory has been created!!!";
    }
    LOG(INFO) << "pose size: " << lPose_.size();
    LOG(INFO) << "refPCMap_.size(): " << refPCMap_.size() << ", TS_history_.size(): " << TS_history_.size();
    saveTrajectory(resultPath_ + "result.txt");
  }
}

bool
esvo_Tracking::refDataTransferring()
{
  // load reference info
  ref_.t_ = refPCMap_.rbegin()->first;


  // ESVO_System_Status_ is updated via shared topic subscription
  if(ESVO_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == IDLE)
    ref_.tr_.setIdentity();

  const auto status = ESVO_System_Status_.load();
  if(status == SystemStatus::WORKING || (status == SystemStatus::INITIALIZATION && ets_ == WORKING))
  {
    if(!getPoseAt(ref_.t_, ref_.tr_, dvs_frame_id_))
    {
      LOG(INFO) << "ESVO_System_Status_: " << to_string(ESVO_System_Status_.load()) << ", ref_.t_: " << ref_.t_.nanoseconds();
      LOG(INFO) << "Logic error ! There must be a pose for the given timestamp, because mapping has been finished.";
      exit(-1);
      return false;
    }
  }

  size_t numPoint = refPCMap_.rbegin()->second->size();
  ref_.vPointXYZPtr_.clear();
  ref_.vPointXYZPtr_.reserve(numPoint);
  auto PointXYZ_begin_it = refPCMap_.rbegin()->second->begin();
  auto PointXYZ_end_it   = refPCMap_.rbegin()->second->end();
  while(PointXYZ_begin_it != PointXYZ_end_it)
  {
    ref_.vPointXYZPtr_.push_back(PointXYZ_begin_it.base());// Copy the pointer of the pointXYZ
    PointXYZ_begin_it++;
  }
  return true;
}

bool
esvo_Tracking::curDataTransferring()
{
  if (!time_offset_.has_value()) {
    LOG(WARNING) << "time_offset not yet received, cannot search events";
    return false;
  }
  rclcpp::Time cur_sensor_time = cur_.t_ + time_offset_.value();

  // load current observation
  auto ev_last_it = EventBuffer_lower_bound(events_left_, cur_sensor_time);
  auto TS_it = TS_history_.rbegin();

  // TS_history may not be updated before the tracking loop excutes the data transfering
  if(cur_.t_.nanoseconds() == TS_it->first.nanoseconds())
    return false;
  cur_.t_ = TS_it->first;
  cur_.pTsObs_ = &TS_it->second;

  // ESVO_System_Status_ is updated via shared topic subscription
  if(ESVO_System_Status_.load() == SystemStatus::INITIALIZATION && ets_ == IDLE)
  {
    cur_.tr_ = ref_.tr_;
//    LOG(INFO) << "(IDLE) Assign cur's ("<< cur_.t_.nanoseconds() << ") pose with ref's at " << ref_.t_.nanoseconds();
    // LOG(INFO) << " " << cur_.tr_.getTransformationMatrix() << " ";
  }

  const auto status = ESVO_System_Status_.load();
  if(status == SystemStatus::WORKING || (status == SystemStatus::INITIALIZATION && ets_ == WORKING))
  {
    cur_.tr_ = Transformation(T_world_cur_);
//    LOG(INFO) << "(WORKING) Assign cur's ("<< cur_.t_.nanoseconds() << ") pose with T_world_cur.";
  }
  // Count the number of events occuring since the last observation.
  rclcpp::Time old_sensor_time = cur_sensor_time;
  cur_sensor_time = cur_.t_ + time_offset_.value();
  auto ev_cur_it = EventBuffer_lower_bound(events_left_, cur_sensor_time);
  cur_.numEventsSinceLastObs_ = std::distance(ev_last_it, ev_cur_it) + 1;
  return true;
}

void esvo_Tracking::reset()
{
  // clear all maintained data
  ets_ = IDLE;
  TS_id_ = 0;
  TS_history_.clear();
  refPCMap_.clear();
  events_left_.clear();
}


/********************** Callback functions *****************************/
void esvo_Tracking::refMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  PointCloud::Ptr PC_ptr(new PointCloud());
  pcl::fromPCLPointCloud2(pcl_pc, *PC_ptr);
  refPCMap_.emplace(msg->header.stamp, PC_ptr);
  while(refPCMap_.size() > REF_HISTORY_LENGTH_)
  {
    auto it = refPCMap_.begin();
    refPCMap_.erase(it);
  }
}

void esvo_Tracking::eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add new ones and remove old ones
  for(const dvs_msgs::msg::Event& e : msg->events)
  {
    events_left_.push_back(e);
    int i = events_left_.size() - 2;
    while(i >= 0 && rclcpp::Time(events_left_[i].ts).nanoseconds() > rclcpp::Time(e.ts).nanoseconds()) // we may have to sort the queue, just in case the raw event messages do not come in a chronological order.
    {
      events_left_[i+1] = events_left_[i];
      i--;
    }
    events_left_[i+1] = e;
  }
  clearEventQueue();
}

void esvo_Tracking::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
  if (events_left_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_left_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_left_.erase(events_left_.begin(), events_left_.begin() + remove_events);
  }
}

void
esvo_Tracking::timeSurfaceCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr& time_surface_left,
  const sensor_msgs::msg::Image::ConstSharedPtr& time_surface_right)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try
  {
    cv_ptr_left  = cv_bridge::toCvCopy(time_surface_left,  sensor_msgs::image_encodings::MONO8);
    cv_ptr_right = cv_bridge::toCvCopy(time_surface_right, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // push back the most current TS.
  rclcpp::Time t_new_ts = time_surface_left->header.stamp;
  TS_history_.emplace(t_new_ts, TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, TS_id_, false));
  TS_id_++;

  // keep TS_history_'s size constant
  while(TS_history_.size() > TS_HISTORY_LENGTH_)
  {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
}

void esvo_Tracking::stampedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Create transform and add to buffer
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header = msg->header;
  transform_stamped.child_frame_id = dvs_frame_id_;
  transform_stamped.transform.translation.x = msg->pose.position.x;
  transform_stamped.transform.translation.y = msg->pose.position.y;
  transform_stamped.transform.translation.z = msg->pose.position.z;
  transform_stamped.transform.rotation = msg->pose.orientation;

  // Use a static transform broadcaster to publish the transform
  static tf2_ros::TransformBroadcaster br(this);
  br.sendTransform(transform_stamped);

  // Also set it in the buffer for lookups
  tf_buffer_->setTransform(transform_stamped, "esvo_tracking", false);
}

bool
esvo_Tracking::getPoseAt(
  const rclcpp::Time &t, esvo_core::Transformation &Tr, const std::string &source_frame)
{
  try
  {
    if(!tf_buffer_->canTransform(world_frame_id_, source_frame, t, rclcpp::Duration::from_seconds(0.0)))
    {
      LOG(WARNING) << t.nanoseconds() << " : Cannot transform";
      return false;
    }
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(world_frame_id_, source_frame, t);
    // Convert geometry_msgs::msg::TransformStamped to tf2::Transform
    tf2::Transform tf2_transform;
    tf2_transform.setOrigin(tf2::Vector3(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z));
    tf2_transform.setRotation(tf2::Quaternion(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w));
    tf::transformTFToKindr(tf2_transform, &Tr);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    LOG(WARNING) << t.nanoseconds() << " : " << ex.what();
    return false;
  }
}

/************ publish results *******************/
void esvo_Tracking::publishPose(const rclcpp::Time &t, Transformation &tr)
{
  auto ps_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>();
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = world_frame_id_;
  ps_ptr->pose.position.x = tr.getPosition()(0);
  ps_ptr->pose.position.y = tr.getPosition()(1);
  ps_ptr->pose.position.z = tr.getPosition()(2);
  ps_ptr->pose.orientation.x = tr.getRotation().x();
  ps_ptr->pose.orientation.y = tr.getRotation().y();
  ps_ptr->pose.orientation.z = tr.getRotation().z();
  ps_ptr->pose.orientation.w = tr.getRotation().w();
  pose_pub_->publish(*ps_ptr);
}

void esvo_Tracking::publishPath(const rclcpp::Time& t, Transformation& tr)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = t;
  ps.header.frame_id = world_frame_id_;
  ps.pose.position.x = tr.getPosition()(0);
  ps.pose.position.y = tr.getPosition()(1);
  ps.pose.position.z = tr.getPosition()(2);
  ps.pose.orientation.x = tr.getRotation().x();
  ps.pose.orientation.y = tr.getRotation().y();
  ps.pose.orientation.z = tr.getRotation().z();
  ps.pose.orientation.w = tr.getRotation().w();

  path_.header.stamp = t;
  path_.header.frame_id = world_frame_id_;
  path_.poses.push_back(ps);
  path_pub_->publish(path_);
}

void
esvo_Tracking::saveTrajectory(const std::string &resultDir)
{
  LOG(INFO) << "Saving trajectory to " << resultDir << " ......";

  std::ofstream  f;
  f.open(resultDir.c_str(), std::ofstream::out);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << resultDir << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;

  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_begin = lPose_.begin();
  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_end = lPose_.end();
  std::list<std::string>::iterator  ts_it_begin = lTimestamp_.begin();

  for(;result_it_begin != result_it_end; result_it_begin++, ts_it_begin++)
  {
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    Rwc_result = (*result_it_begin).block<3,3>(0,0);
    twc_result = (*result_it_begin).block<3,1>(0,3);
    Eigen::Quaterniond q(Rwc_result);
    f << *ts_it_begin << " " << std::setprecision(9) << twc_result.transpose() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  f.close();
  LOG(INFO) << "Saving trajectory to " << resultDir << ". Done !!!!!!.";
}

}// namespace esvo_core
