cmake_minimum_required(VERSION 3.12)
project(steering_monitor_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(steering_monitor_node src/steering_monitor_node.cpp)
ament_target_dependencies(steering_monitor_node rclcpp std_msgs)

install(TARGETS steering_monitor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

# 옵션: colcon이 CATKIN_* 변수를 넘기는 환경에서 경고 제거용(미사용 변수 참조)
# 이 블록은 빌드 동작에 영향을 주지 않습니다.
set(_unused_catkin_install_into_prefix_root "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
set(_unused_catkin_symlink_install "${CATKIN_SYMLINK_INSTALL}")
