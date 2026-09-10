cmake_minimum_required(VERSION 3.8)
project(longitudinal_mpc)

find_package(ament_cmake REQUIRED)
find_package(kaiev26_msgs REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED CONFIG)
find_package(rclcpp REQUIRED)

add_library(longitudinal_mpc_core STATIC
  src/osqp_mpc.cpp
)
target_compile_features(longitudinal_mpc_core PUBLIC cxx_std_17)
target_include_directories(longitudinal_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(longitudinal_mpc_core PUBLIC osqp::osqp)

add_executable(longitudinal_mpc_node
  src/longitudinal_mpc_node.cpp
)
target_compile_features(longitudinal_mpc_node PUBLIC cxx_std_17)
target_link_libraries(longitudinal_mpc_node longitudinal_mpc_core)
ament_target_dependencies(longitudinal_mpc_node kaiev26_msgs rclcpp)

install(TARGETS longitudinal_mpc_core longitudinal_mpc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(longitudinal_mpc_osqp_test test/test_osqp_mpc.cpp)
  target_link_libraries(longitudinal_mpc_osqp_test longitudinal_mpc_core)
endif()

ament_package()
