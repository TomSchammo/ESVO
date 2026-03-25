#include <esvo_time_surface/TimeSurface.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<esvo_time_surface::TimeSurface>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
