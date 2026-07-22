cmake_minimum_required(VERSION 3.8)
project(lateral_ctrl) # package name 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(steering_control_node src/steering_control_node.cpp)
ament_target_dependencies(steering_control_node rclcpp std_msgs)

add_executable(steering_feedback_node src/steering_feedback_node.cpp)
ament_target_dependencies(steering_feedback_node rclcpp std_msgs)

add_executable(deadzone_measure_node src/deadzone_measure_node.cpp)
ament_target_dependencies(deadzone_measure_node rclcpp std_msgs)

install(TARGETS
  steering_control_node
  steering_feedback_node
  deadzone_measure_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
