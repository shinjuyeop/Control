cmake_minimum_required(VERSION 3.16)
project(dashboard_renderer LANGUAGES CXX)
set(DASHBOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
add_library(dashboard_renderer SHARED
  display_renderer.cpp
  ${DASHBOARD_ROOT}/dashboard_ui.cpp
  ${DASHBOARD_ROOT}/manual_tft_ui.cpp
  ${DASHBOARD_ROOT}/st7796_lcd.cpp)
target_compile_features(dashboard_renderer PRIVATE cxx_std_17)
target_compile_options(dashboard_renderer PRIVATE -Wall -Wextra)
target_include_directories(dashboard_renderer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${DASHBOARD_ROOT})
install(TARGETS dashboard_renderer LIBRARY DESTINATION dashboard)
