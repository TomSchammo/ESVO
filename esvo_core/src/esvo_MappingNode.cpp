#include <esvo_core/esvo_Mapping.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<esvo_core::esvo_Mapping>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

