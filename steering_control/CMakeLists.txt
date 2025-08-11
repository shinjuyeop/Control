cmake_minimum_required(VERSION 3.12)
project(steering_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include 디렉토리 추가
include_directories(
  include
)

add_executable(steering_control_node src/steering_control_node.cpp)
add_executable(steering_feedback_node src/steering_feedback_node.cpp)
add_executable(steering_command_publisher_node src/steering_command_publisher_node.cpp)

ament_target_dependencies(steering_control_node rclcpp std_msgs)
ament_target_dependencies(steering_feedback_node rclcpp std_msgs)
ament_target_dependencies(steering_command_publisher_node rclcpp std_msgs)

install(TARGETS
  steering_control_node
  steering_feedback_node
  steering_command_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

