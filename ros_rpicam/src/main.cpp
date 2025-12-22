#include <rclcpp/rclcpp.hpp>
#include "ros_rpicam/rpi_cam_node.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<rpicamera::rpiCamNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
