cmake_minimum_required(VERSION 3.8)
project(longitudinal_ctrl)

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
find_package(geometry_msgs REQUIRED)
find_package(kaiev26_msgs REQUIRED)

# 제어 수학은 ROS 의존 없는 정적 라이브러리로 분리해 host gtest로 시험한다.
# lateral_ctrl의 steering_core와 같은 구조.
add_library(longitudinal_core src/speed_torque_controller.cpp)
target_include_directories(longitudinal_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(longitudinal_node src/longitudinal_actuator_node.cpp)
target_link_libraries(longitudinal_node longitudinal_core)
ament_target_dependencies(longitudinal_node rclcpp std_msgs geometry_msgs kaiev26_msgs)

add_executable(longitudinal_actuator_node src/longitudinal_actuator_main.cpp)
target_link_libraries(longitudinal_actuator_node longitudinal_node)
ament_target_dependencies(longitudinal_actuator_node rclcpp)

install(TARGETS longitudinal_core longitudinal_node longitudinal_actuator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_speed_torque_controller test/test_speed_torque_controller.cpp)
  target_link_libraries(test_speed_torque_controller longitudinal_core)

  ament_add_gtest(test_longitudinal_actuator_node test/test_longitudinal_actuator_node.cpp)
  target_link_libraries(test_longitudinal_actuator_node longitudinal_node)
  ament_target_dependencies(test_longitudinal_actuator_node rclcpp std_msgs geometry_msgs kaiev26_msgs)
endif()

ament_export_include_directories(include)
ament_package()
