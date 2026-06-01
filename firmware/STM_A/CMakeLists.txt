cmake_minimum_required(VERSION 3.22)

# ★ cube-cmake 컴파일러 테스트 링크 오류 방지
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM_A)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ─── MCU 컴파일/링크 플래그 (STM_B와 동일하게 명시) ───────────────────────────
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
)
add_compile_options(${MCU_FLAGS})
add_link_options(${MCU_FLAGS} -Wl,--gc-sections)



# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ─── microROS 설정 ────────────────────────────────────────────────────────────
set(MICROROS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/micro_ros)

# extra_sources 전체 수집 후 미사용 transport 제외
file(GLOB_RECURSE MICROROS_EXTRA_SOURCES "${MICROROS_DIR}/extra_sources/*.c")
list(FILTER MICROROS_EXTRA_SOURCES EXCLUDE REGEX "usb_cdc_transport\\.c$")
list(FILTER MICROROS_EXTRA_SOURCES EXCLUDE REGEX "udp_transport\\.c$")
list(FILTER MICROROS_EXTRA_SOURCES EXCLUDE REGEX "it_transport\\.c$")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MICROROS_EXTRA_SOURCES}
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    ${MICROROS_DIR}/microros_include
    ${MICROROS_DIR}/extra_sources
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    ${MICROROS_DIR}/libmicroros.a
    stm32cubemx
    m
    c
    nosys
)