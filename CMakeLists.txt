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
set(CMAKE_PROJECT_NAME NYUSH_Infantry)

# ------------------------------------------------------------
# Robot configuration type
# (Override from CLI or VS Code task with -DROBOT_TYPE=...)
# ------------------------------------------------------------
set(ROBOT_TYPE "infantry_standard" CACHE STRING "Robot configuration type")

# Optional: show available values in GUIs (ccmake/cmake-gui)
set_property(CACHE ROBOT_TYPE PROPERTY STRINGS
    infantry_standard
    sentry_swerve
    some_other_robot
)

# Optional: validate to fail fast on typos
set(_valid_robot_types infantry_standard sentry_swerve some_other_robot)
list(FIND _valid_robot_types "${ROBOT_TYPE}" _robot_type_idx)
if(_robot_type_idx EQUAL -1)
    message(FATAL_ERROR "Invalid ROBOT_TYPE='${ROBOT_TYPE}'. Valid: ${_valid_robot_types}")
endif()

message(STATUS "Selected ROBOT_TYPE: ${ROBOT_TYPE}")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ------------------------------------------------------------
# Executable target
# ------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME})

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-u,_printf_float
    -mthumb
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/ARM/DSP/Lib
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/buzzer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Inc
    config
    modules
    modules/message_center
    modules/can_comm
    modules/debug_print
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    application
    application/chassis
    application/shoot
    application/gimbal
    application/cmd
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Middlewares/ST/ARM/DSP/Include
)

# Add project symbols (macros) for executable
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mthumb
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ------------------------------------------------------------
# Layered static libraries
# ------------------------------------------------------------

# modules library
add_library(modules STATIC
    config/robot_config.c
    modules/message_center/message_center.c
    modules/algorithm/pid.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/motor/motor_driver.c
    modules/remote/bsp_rc.c
    modules/remote/remote_control.c
    modules/imu/bmi088driver.c
    modules/imu/bmi088middleware.c
    modules/imu/gyro_data.c
    modules/imu/wt61c.c
    modules/can_comm/can_manager.c
    modules/can_comm/motor_registry.c
    modules/debug_print/printing.c
    modules/vision_comm/crc8.c
    modules/vision_comm/crc16.c
    modules/vision_comm/seasky_protocol.c
    modules/vision_comm/vision_comm.c
)

target_include_directories(modules PUBLIC
    config
    modules
    modules/message_center
    modules/can_comm
    modules/debug_print
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    modules/vision_comm
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Middlewares/ST/ARM/DSP/Include
)

# Apply ROBOT_TYPE macro to modules compilation units too
target_compile_definitions(modules PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

# Inherit STM32CubeMX interface includes and compile definitions (e.g., USE_HAL_DRIVER, STM32F407xx)
target_link_libraries(modules PUBLIC stm32cubemx)
target_compile_options(modules PUBLIC -mthumb)

# application library
add_library(application STATIC
    application/cmd/cmd_controller.c
    application/chassis/chassis_controller.c
    application/shoot/shooter_controller.c
    application/gimbal/gimbal_controller.c
)

target_include_directories(application PUBLIC
    config
    application
    application/cmd
    application/chassis
    application/shoot
    application/gimbal
    modules
    modules/message_center
    modules/can_comm
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    modules/vision_comm
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
)

# Apply ROBOT_TYPE macro to application compilation units too
target_compile_definitions(application PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

# Inherit STM32CubeMX interface includes and compile definitions (e.g., USE_HAL_DRIVER, STM32F407xx)
target_link_libraries(application PUBLIC stm32cubemx modules)
target_compile_options(application PUBLIC -mthumb)

# ------------------------------------------------------------
# Link executable to libraries
# ------------------------------------------------------------
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    modules
    application
    m
    arm_cortexM4lf_math
)
