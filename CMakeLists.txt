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
set(CMAKE_PROJECT_NAME target_vehicle)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP/bsp_fdcan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP/bsp_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/ibus_decoder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/Motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/motor_6220.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/motor_3508.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/motor_zdt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/lcd_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver/lcd_key.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_can_recv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_ibus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_gimbal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_chassis.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_zdt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_flash_lcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_cmd_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks/task_safety.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Middleware/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Middleware/chassis_kinematics.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/AppTasks
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Middleware
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
