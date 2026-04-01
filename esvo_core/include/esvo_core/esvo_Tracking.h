#ifndef ESVO_CORE_TRACKING_H
#define ESVO_CORE_TRACKING_H

#include <nav_msgs/msg/path.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/core/RegProblemLM.h>
#include <esvo_core/core/RegProblemSolverLM.h>
#include <esvo_core/tools/utils.h>
#include <esvo_core/tools/Visualization.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <map>
#include <deque>
#include <mutex>
#include <future>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

namespace esvo_core
{
using namespace core;
enum TrackingStatus
{
  IDLE,
  WORKING
};

class esvo_Tracking : public rclcpp::Node
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  esvo_Tracking();
  virtual ~esvo_Tracking();

  // functions regarding tracking
  void TrackingLoop();
  bool refDataTransferring();
  bool curDataTransferring();// These two data transferring functions are decoupled because the data are not updated at the same frequency.

  // topic callback functions
  void refMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timeSurfaceCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& time_surface_left,
    const sensor_msgs::msg::Image::ConstSharedPtr& time_surface_right);
  void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg);

  // results
  void publishPose(const rclcpp::Time& t, Transformation& tr);
  void publishPath(const rclcpp::Time& t, Transformation& tr);
  void saveTrajectory(const std::string &resultDir);

  // utils
  void reset();
  void clearEventQueue();
  void stampedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool getPoseAt(
    const rclcpp::Time &t,
    esvo_core::Transformation &Tr,// T_world_something
    const std::string& source_frame );

  private:
  // subscribers and publishers
  rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr events_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> TS_left_sub_, TS_right_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stampedPose_sub_;
  image_transport::Publisher reprojMap_pub_left_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;

  // results
  nav_msgs::msg::Path path_;
  std::list<Eigen::Matrix<double,4,4>, Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > > lPose_;
  std::list<std::string> lTimestamp_;

  // Time Surface sync policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ExactSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> TS_sync_;

  // offline data
  std::string dvs_frame_id_;
  std::string world_frame_id_;
  std::string calibInfoDir_;
  CameraSystem::Ptr camSysPtr_;

  // inter-thread management
  std::mutex data_mutex_;

  // online data
  EventQueue events_left_;
  TimeSurfaceHistory TS_history_;
  size_t TS_id_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  RefPointCloudMap refPCMap_;
  RefFrame ref_;
  CurFrame cur_;

  /**** offline parameters ***/
  size_t tracking_rate_hz_;
  size_t TS_HISTORY_LENGTH_;
  size_t REF_HISTORY_LENGTH_;
  bool bSaveTrajectory_;
  bool bVisualizeTrajectory_;
  std::string resultPath_;

  Eigen::Matrix<double, 4, 4> T_world_ref_;
  Eigen::Matrix<double, 4, 4> T_world_cur_;

  /*** system objects ***/
  RegProblemType rpType_;
  TrackingStatus ets_;
  std::string ESVO_System_Status_;
  RegProblemConfig::Ptr rpConfigPtr_;
  RegProblemSolverLM rpSolver_;
};
}


#endif //ESVO_CORE_TRACKING_H
