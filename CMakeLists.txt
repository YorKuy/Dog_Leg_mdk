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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME New_Chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C ASM CXX)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# stm32f4xx_it.c/my_math.c include C++ headers (RM_Lib.h), must compile as C++.
# Core/Src/main.c now remains a CubeMX-compatible C hardware entry layer.
set_source_files_properties(Core/Src/stm32f4xx_it.c RM2023_Lib_V1.2/my_math.c PROPERTIES LANGUAGE CXX)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # RM2023_Lib_V1.2
    RM2023_Lib_V1.2/RM_Lib.cpp
    RM2023_Lib_V1.2/CP_System.c
    RM2023_Lib_V1.2/communication.c
    RM2023_Lib_V1.2/my_math.c
    Motor_Lib/DM.cpp
    # Math_Lib (PID.cpp removed: PID_class already in RM_Lib.h)
    Math_Lib/SMC.cpp
    # IMU
    IMU/imu_data_decode.c
    IMU/packet.c
    # CH010
    CH010/hipnuc_dec.c
    CH010/nmea_dec.c
    CH010/example_data.c
    # Control_Lib
    Control_Lib/leg.cpp
    Control_Lib/leg_balance.cpp
    Control_Lib/leg_output.cpp
    Control_Lib/leg_height.cpp
    # App_Lib
    App_Lib/app_chassis.cpp
    App_Lib/main.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    RM2023_Lib_V1.2
    Motor_Lib
    Math_Lib
    IMU
    CH010
    Control_Lib
    App_Lib
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    DOG_LEG_USE_CPP_MAIN
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# OpenOCD helper targets. The top-level Makefile calls OpenOCD directly, while
# these targets keep the same actions available from CMake/Ninja.
find_program(OPENOCD_EXECUTABLE openocd)
if(NOT OPENOCD_EXECUTABLE)
    set(OPENOCD_EXECUTABLE openocd)
endif()

set(OPENOCD_CONFIG ${CMAKE_SOURCE_DIR}/openocd.cfg)

add_custom_target(build_flash
    COMMAND ${OPENOCD_EXECUTABLE} -f ${OPENOCD_CONFIG}
        -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flash ${CMAKE_PROJECT_NAME} to STM32F405"
)

add_custom_target(flash
    COMMAND ${OPENOCD_EXECUTABLE} -f ${OPENOCD_CONFIG}
        -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flash ${CMAKE_PROJECT_NAME} to STM32F405"
)

add_custom_target(erase
    COMMAND ${OPENOCD_EXECUTABLE} -f ${OPENOCD_CONFIG}
        -c "init; reset halt; stm32f4x mass_erase 0; reset run; exit"
    COMMENT "Erase STM32F405 flash"
)

add_custom_target(reset
    COMMAND ${OPENOCD_EXECUTABLE} -f ${OPENOCD_CONFIG}
        -c "init; reset run; exit"
    COMMENT "Reset STM32F405"
)
