cmake_minimum_required(VERSION 3.12)
project(motor_control_cpp)

# ─────── compile options ────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────── dependencies ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)   # 파라미터 콜백용 (정말 쓸 때만 유지)

# ─────── executables ────────────────────────────────────────────────
# 1 속도 퍼블리셔
add_executable(speed_publisher src/speed_publisher.cpp)
ament_target_dependencies(speed_publisher
  rclcpp
  std_msgs
  rcl_interfaces)

# 2 CAN 송신 노드
add_executable(can_speed_sender src/can_speed_sender.cpp)
ament_target_dependencies(can_speed_sender
  rclcpp
  std_msgs)

# 3 CAN 수신 노드 (신규)
add_executable(can_speed_receiver src/can_speed_receiver.cpp)
ament_target_dependencies(can_speed_receiver
  rclcpp
  std_msgs)

# ─────── install  ───────────────────────────────────────────────────
install(TARGETS
  speed_publisher
  can_speed_sender
  can_speed_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# ─────── optional testing  ───────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
