cmake_minimum_required(VERSION 3.16)
project(dashboard_renderer LANGUAGES CXX)
set(DASHBOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(DASHBOARD_SOURCE "${DASHBOARD_ROOT}/src")
set(DASHBOARD_HOST_NATIVE "${DASHBOARD_ROOT}/simulator/native")
add_library(dashboard_renderer SHARED
  display_renderer.cpp
  ${DASHBOARD_HOST_NATIVE}/host_runtime.cpp
  ${DASHBOARD_SOURCE}/auto_tft_ui.cpp
  ${DASHBOARD_SOURCE}/dashboard_timing.cpp
  ${DASHBOARD_SOURCE}/dashboard_ui.cpp
  ${DASHBOARD_SOURCE}/manual_tft_ui.cpp
  ${DASHBOARD_SOURCE}/st7796_lcd.cpp
  ${DASHBOARD_SOURCE}/tft_font_renderer.cpp)
target_compile_features(dashboard_renderer PRIVATE cxx_std_17)
target_compile_options(dashboard_renderer PRIVATE -Wall -Wextra)
target_compile_definitions(dashboard_renderer PRIVATE DASHBOARD_HOST)
target_include_directories(dashboard_renderer PRIVATE
  ${DASHBOARD_HOST_NATIVE}
  ${DASHBOARD_SOURCE}/include
  ${DASHBOARD_ROOT})
install(TARGETS dashboard_renderer LIBRARY DESTINATION dashboard)
