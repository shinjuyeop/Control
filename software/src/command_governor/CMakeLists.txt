cmake_minimum_required(VERSION 3.8)
project(command_governor)

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
find_package(kaiev26_msgs REQUIRED)

# 판정 로직은 ROS 의존 없는 정적 라이브러리로 분리해 host gtest로 시험한다.
# lateral_ctrl의 steering_core와 같은 구조.
add_library(governor_core
  src/governor_core.cpp
)
target_include_directories(governor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(command_governor_node src/command_governor_node.cpp)
target_include_directories(command_governor_node PRIVATE include)
target_link_libraries(command_governor_node governor_core)
ament_target_dependencies(command_governor_node rclcpp kaiev26_msgs)

install(TARGETS governor_core command_governor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_governor_core test/test_governor_core.cpp)
  target_link_libraries(test_governor_core governor_core)
  target_include_directories(test_governor_core PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
