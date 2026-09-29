cmake_minimum_required(VERSION 3.8)
project(lateral_ctrl) # package name 

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
find_package(steering_mpc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(kaiev26_msgs REQUIRED)

add_library(steering_core
  src/keya_can_driver.cpp
  src/steering_angle_converter.cpp
  src/wheel_angle_tracking_controller.cpp
  src/torque_steering_controller.cpp
)
target_include_directories(steering_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(steering_node src/steering_actuator_node.cpp)
target_link_libraries(steering_node steering_core steering_mpc::steering_mpc_core)
ament_target_dependencies(steering_node rclcpp std_msgs kaiev26_msgs diagnostic_msgs steering_mpc)
add_executable(steering_actuator_node src/steering_actuator_main.cpp)
target_link_libraries(steering_actuator_node steering_node)
ament_target_dependencies(steering_actuator_node rclcpp)

install(TARGETS steering_core steering_node steering_actuator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md TORQUE_CONTROL.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_keya_can_driver test/test_keya_can_driver.cpp)
  target_link_libraries(test_keya_can_driver steering_core)
  target_include_directories(test_keya_can_driver PRIVATE include src)

  ament_add_gtest(test_steering_angle_converter test/test_steering_angle_converter.cpp)
  target_link_libraries(test_steering_angle_converter steering_core)
  target_include_directories(test_steering_angle_converter PRIVATE include)

  ament_add_gtest(
    test_wheel_angle_tracking_controller test/test_wheel_angle_tracking_controller.cpp)
  target_link_libraries(test_wheel_angle_tracking_controller steering_core)
  target_include_directories(test_wheel_angle_tracking_controller PRIVATE include)

  ament_add_gtest(test_steering_safety_logic test/test_steering_safety_logic.cpp)
  target_include_directories(test_steering_safety_logic PRIVATE include)

  ament_add_gtest(test_torque_steering_controller test/test_torque_steering_controller.cpp)
  target_link_libraries(test_torque_steering_controller steering_core)

  ament_add_gtest(test_steering_actuator_node test/test_steering_actuator_node.cpp)
  target_link_libraries(test_steering_actuator_node steering_node)
  ament_target_dependencies(test_steering_actuator_node rclcpp std_msgs kaiev26_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(steering_core)
ament_package()
