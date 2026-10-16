cmake_minimum_required(VERSION 3.22)

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
set(CMAKE_PROJECT_NAME C10B_Test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
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
    stm32cubemx

    # Add user defined libraries
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/motor.c
    Core/Src/Modules/encoder.c
    Core/Src/Modules/speed_control.c
    Core/Src/Modules/button.c
    Core/Src/Modules/car.c
    Core/Src/Modules/task_key.c
    Core/Src/Modules/position_stop.c
    Core/Src/Modules/lap_counter.c
    Core/Src/Modules/task2.c
    Core/Src/Modules/oled.c
    Core/Src/Modules/line_follow.c
    Core/Src/Modules/line_sensor.c
    Core/Src/Modules/line_calibration.c
    Core/Src/Modules/line_calibration_store.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/imu_heading.c
    Core/Src/Modules/imu_mpu6050.c
    Core/Src/Modules/imu_test.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/mode2.c
    Core/Src/Modules/mode2_control.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Modules/bt_protocol.c
    Core/Src/Modules/bt_uart.c
    Core/Src/Modules/task4.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/Modules/bt_probe.c)
