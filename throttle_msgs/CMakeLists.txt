cmake_minimum_required(VERSION 3.8)
project(throttle_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### ── 1. 의존 패키지 ──────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # ★ 추가
find_package(std_msgs REQUIRED)

### ── 2. 메시지-인터페이스 생성 지시 ───────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThrottleData.msg"
  DEPENDENCIES std_msgs
)

### ── 3. 테스트(옵션) ──────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
