cmake_minimum_required(VERSION 3.8)
project(lateral_ctrl)

if(POLICY CMP0148)
  # ROS 2 Humble's rosidl_generator_py still uses FindPythonInterp/FindPythonLibs.
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kaiev26_msgs REQUIRED)

add_library(steering_core
  src/keya_can_driver.cpp
  src/steering_angle_converter.cpp
  src/wheel_angle_tracking_controller.cpp
)
target_include_directories(steering_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(steering_actuator_node src/steering_actuator_node.cpp)
target_include_directories(steering_actuator_node PRIVATE include)
target_link_libraries(steering_actuator_node steering_core)
ament_target_dependencies(steering_actuator_node rclcpp std_msgs kaiev26_msgs)

add_executable(steering_bench_test_node src/steering_bench_test_node.cpp)
target_include_directories(steering_bench_test_node PRIVATE include)
ament_target_dependencies(steering_bench_test_node rclcpp std_msgs)

install(TARGETS steering_core steering_actuator_node steering_bench_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_keya_can_driver test/test_keya_can_driver.cpp)
  target_link_libraries(test_keya_can_driver steering_core)
  target_include_directories(test_keya_can_driver PRIVATE include)

  ament_add_gtest(test_steering_angle_converter test/test_steering_angle_converter.cpp)
  target_link_libraries(test_steering_angle_converter steering_core)
  target_include_directories(test_steering_angle_converter PRIVATE include)

  ament_add_gtest(
    test_wheel_angle_tracking_controller test/test_wheel_angle_tracking_controller.cpp)
  target_link_libraries(test_wheel_angle_tracking_controller steering_core)
  target_include_directories(test_wheel_angle_tracking_controller PRIVATE include)

  ament_add_gtest(test_steering_safety_logic test/test_steering_safety_logic.cpp)
  target_include_directories(test_steering_safety_logic PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
