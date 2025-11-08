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

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-u,_printf_float
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -mthumb)
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Src/buzzer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Inc
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
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -mthumb)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    modules
    application
    m
)

# Create layered static libraries (placeholders for initial structure)
add_library(modules STATIC
    modules/message_center/message_center.c
    modules/algorithm/pid.c
    modules/motor/gm6020_motor.c
    modules/remote/bsp_rc.c
    modules/remote/remote_control.c
    modules/imu/bmi088driver.c
    modules/imu/bmi088middleware.c
    modules/imu/gyro_data.c
    modules/imu/wt61c.c
    modules/can_comm/can_manager.c
    modules/debug_print/printing.c
)

target_include_directories(modules PUBLIC
    modules
    modules/message_center
    modules/can_comm
    modules/debug_print
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
)

# Inherit STM32CubeMX interface includes and compile definitions (e.g., USE_HAL_DRIVER, STM32F407xx)
target_link_libraries(modules PUBLIC stm32cubemx)
target_compile_options(modules PUBLIC -mthumb)

add_library(application STATIC
    application/cmd/cmd_controller.c
    application/chassis/chassis_controller.c
    application/shoot/shooter_controller.c
    application/gimbal/gimbal_controller.c
)

target_include_directories(application PUBLIC
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
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
)

# Inherit STM32CubeMX interface includes and compile definitions (e.g., USE_HAL_DRIVER, STM32F407xx)
target_link_libraries(application PUBLIC stm32cubemx modules)
target_compile_options(application PUBLIC -mthumb)
