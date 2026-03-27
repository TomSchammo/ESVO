#ifndef ESVO_CORE_TOOLS_UTILS_H
#define ESVO_CORE_TOOLS_UTILS_H

#include <Eigen/Eigen>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <kindr/minimal/quat-transformation.h>

#include <dvs_msgs/msg/event.hpp>
#include <dvs_msgs/msg/event_array.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <esvo_core/container/SmartGrid.h>
#include <esvo_core/container/DepthPoint.h>
#include <esvo_core/tools/TicToc.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
namespace esvo_core
{
namespace tools
{
// TUNE this according to your platform's computational capability.
#define NUM_THREAD_TRACKING 1
#define NUM_THREAD_MAPPING 4

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Custom comparator for rclcpp::Time to avoid "can't compare times with different time sources"
struct RclcppTimeCmp
{
  bool operator()(const rclcpp::Time &a, const rclcpp::Time &b) const
  {
    return a.nanoseconds() < b.nanoseconds();
  }
};

using RefPointCloudMap = std::map<rclcpp::Time, PointCloud::Ptr, RclcppTimeCmp>;

using Transformation = kindr::minimal::QuatTransformation;

inline static std::vector<dvs_msgs::msg::Event*>::iterator EventVecPtr_lower_bound(
  std::vector<dvs_msgs::msg::Event*>& vEventPtr, rclcpp::Time& t)
{
  return std::lower_bound(vEventPtr.begin(), vEventPtr.end(), t,
                          [](const dvs_msgs::msg::Event* e, const rclcpp::Time &t){return rclcpp::Time(e->ts).seconds() < t.seconds();});
}

using EventQueue = std::deque<dvs_msgs::msg::Event>;
inline static EventQueue::iterator EventBuffer_lower_bound(
  EventQueue& eb, rclcpp::Time& t)
{
  return std::lower_bound(eb.begin(), eb.end(), t,
    [](const dvs_msgs::msg::Event & e, const rclcpp::Time & t) {return rclcpp::Time(e.ts).seconds() < t.seconds();});
}

inline static EventQueue::iterator EventBuffer_upper_bound(
  EventQueue& eb, rclcpp::Time& t)
{
  return std::upper_bound(eb.begin(), eb.end(), t,
    [](const rclcpp::Time & t, const dvs_msgs::msg::Event & e) {return t.seconds() < rclcpp::Time(e.ts).seconds();});
}

using StampTransformationMap = std::map<rclcpp::Time, tools::Transformation, RclcppTimeCmp>;
inline static StampTransformationMap::iterator StampTransformationMap_lower_bound(
  StampTransformationMap& stm, rclcpp::Time& t)
{
  return std::lower_bound(stm.begin(), stm.end(), t,
    [](const std::pair<rclcpp::Time, tools::Transformation>& st, const rclcpp::Time& t){return st.first.seconds() < t.seconds();});
}

/******************* Used by Block Match ********************/
static inline void meanStdDev(
  Eigen::MatrixXd& patch,
  double& mean, double& sigma)
{
  size_t numElement = patch.rows() * patch.cols();
  mean = patch.array().sum() / numElement;
  Eigen::MatrixXd sub = patch.array() - mean;
  sigma = sqrt((sub.array() * sub.array()).sum() / numElement) + 1e-6;
}

static inline void normalizePatch(
  Eigen::MatrixXd& patch_src,
  Eigen::MatrixXd& patch_dst)
{
  double mean = 0;
  double sigma = 0;
  meanStdDev(patch_src,mean,sigma);
  patch_dst = (patch_src.array() - mean) / sigma;
}

// recursively create a directory
static inline void _mkdir(const char *dir)
{
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",dir);
  len = strlen(tmp);
  if(tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      mkdir(tmp, S_IRWXU);
      *p = '/';
    }
  mkdir(tmp, S_IRWXU);
}

}// tools
}// esvo_core
#endif //ESVO_CORE_TOOLS_UTILS_H
