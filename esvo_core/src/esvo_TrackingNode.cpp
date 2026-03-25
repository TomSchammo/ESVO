#include <esvo_core/esvo_Tracking.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esvo_core::esvo_Tracking>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

