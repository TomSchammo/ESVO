#ifndef esvo_time_surface_H_
#define esvo_time_surface_H_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>

#include <deque>
#include <mutex>
#include <Eigen/Eigen>

namespace esvo_time_surface
{
#define NUM_THREAD_TS 1
using EventQueue = std::deque<dvs_msgs::msg::Event>;

class EventQueueMat
{
public:
  EventQueueMat(int width, int height, int queueLen)
  {
    width_ = width;
    height_ = height;
    queueLen_ = queueLen;
    eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
  }

  void insertEvent(const dvs_msgs::msg::Event& e)
  {
    if(!insideImage(e.x, e.y))
      return;
    else
    {
      EventQueue& eq = getEventQueue(e.x, e.y);
      eq.push_back(e);
      while(eq.size() > queueLen_)
        eq.pop_front();
    }
  }

  bool getMostRecentEventBeforeT(
    const size_t x,
    const size_t y,
    const rclcpp::Time& t,
    dvs_msgs::msg::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;

    for(auto it = eq.rbegin(); it != eq.rend(); ++it)
    {
      const dvs_msgs::msg::Event& e = *it;
      if(rclcpp::Time(e.ts).nanoseconds() < t.nanoseconds())
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  void clear()
  {
    eqMat_.clear();
  }

  bool insideImage(const size_t x, const size_t y)
  {
    return !(x < 0 || x >= width_ || y < 0 || y >= height_);
  }

  inline EventQueue& getEventQueue(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y];
  }

  size_t width_;
  size_t height_;
  size_t queueLen_;
  std::vector<EventQueue> eqMat_;
};

class TimeSurface : public rclcpp::Node
{
  struct Job
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EventQueueMat* pEventQueueMat_;
    cv::Mat* pTimeSurface_;
    size_t start_col_, end_col_;
    size_t start_row_, end_row_;
    size_t i_thread_;
    rclcpp::Time external_sync_time_;
    double decay_sec_;
  };

public:
  TimeSurface();
  virtual ~TimeSurface();

private:
  // core
  void init(int width, int height);
  void createTimeSurfaceAtTime(const rclcpp::Time& external_sync_time);// single thread version (This is enough for DAVIS240C and DAVIS346)
  void createTimeSurfaceAtTime_hyperthread(const rclcpp::Time& external_sync_time); // hyper thread version (This is for higher resolution)
  void thread(Job& job);

  // callbacks
  void syncCallback(const builtin_interfaces::msg::Time::SharedPtr msg);
  void eventsCallback(const dvs_msgs::msg::EventArray::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // utils
  void clearEventQueue();

  // calibration parameters
  cv::Mat camera_matrix_, dist_coeffs_;
  cv::Mat rectification_matrix_, projection_matrix_;
  std::string distortion_model_;
  cv::Mat undistort_map1_, undistort_map2_;
  Eigen::Matrix2Xd precomputed_rectified_points_;

  // sub & pub
  rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sync_topic_;
  image_transport::Publisher time_surface_pub_;
  rclcpp::Subscription<builtin_interfaces::msg::Duration>::SharedPtr time_offset_sub_;

  // online parameters
  bool bCamInfoAvailable_;
  bool bUse_Sim_Time_;
  cv::Size sensor_size_;
  rclcpp::Time sync_time_;
  bool bSensorInitialized_;

  // time sync
  rclcpp::Duration time_offset_;

  // offline parameters
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  int max_event_queue_length_;
  int events_maintained_size_;

  // containers
  EventQueue events_;
  std::shared_ptr<EventQueueMat> pEventQueueMat_;

  // thread mutex
  std::mutex data_mutex_;

  // Time Surface Mode
  // Backward: First Apply exp decay on the raw image plane, then get the value
  //           at each pixel in the rectified image plane by looking up the
  //           corresponding one (float coordinates) with bi-linear interpolation.
  // Forward: First warp the raw events to the rectified image plane, then
  //          apply the exp decay on the four neighbouring (involved) pixel coordinate.
  enum TimeSurfaceMode
  {
    BACKWARD,// used in the T-RO20 submission
    FORWARD
  } time_surface_mode_;
};
} // namespace esvo_time_surface
#endif // esvo_time_surface_H_
