#include "longitudinal_ctrl/longitudinal_actuator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<longitudinal_ctrl::LongitudinalActuatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("longitudinal_actuator_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
