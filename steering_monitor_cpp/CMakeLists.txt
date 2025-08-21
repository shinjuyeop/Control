cmake_minimum_required(VERSION 3.5)
project(steering_monitor_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(steering_monitor_node src/steering_monitor_node.cpp)
ament_target_dependencies(steering_monitor_node rclcpp std_msgs)

install(TARGETS steering_monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
