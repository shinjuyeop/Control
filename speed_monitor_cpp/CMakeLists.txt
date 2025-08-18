cmake_minimum_required(VERSION 3.12)
project(speed_monitor_cpp)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 실행 파일
add_executable(speed_monitor_node src/speed_monitor_node.cpp)
ament_target_dependencies(speed_monitor_node rclcpp std_msgs)

# 설치
install(TARGETS speed_monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
