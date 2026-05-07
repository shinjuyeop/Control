cmake_minimum_required(VERSION 3.16)
project(STM_A C ASM)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -O0 -g3 -Wall")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} --specs=nosys.specs -Wl,--gc-sections")

add_executable(${PROJECT_NAME}.elf src/main.c)
