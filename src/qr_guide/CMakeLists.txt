cmake_minimum_required(VERSION 3.14)
project(qr_guide)

set(ROBOT_TYPE Go1)
set(PLATFORM arm64)
set(SIMULATION OFF)
set(REAL_ROBOT ON)
set(DEBUG OFF)
set(MOVE_BASE OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()
if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_compile_definitions(ROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_compile_definitions(ROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(REAL_ROBOT)
    add_compile_definitions(COMPILE_WITH_REAL_ROBOT)
endif()
if(MOVE_BASE)
    add_compile_definitions(COMPILE_WITH_MOVE_BASE)
endif()
if(DEBUG)
    add_compile_definitions(COMPILE_DEBUG)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(UNITREE_ACTUATOR_SDK_ROOT "/home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main" CACHE PATH "Local Unitree actuator SDK root")
set(UNITREE_ACTUATOR_SDK_INCLUDE_DIR "${UNITREE_ACTUATOR_SDK_ROOT}/include")
set(UNITREE_ACTUATOR_SDK_LIBRARY "${UNITREE_ACTUATOR_SDK_ROOT}/lib/libUnitreeMotorSDK_Arm64.so")

if(NOT EXISTS "${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}/unitreeMotor/unitreeMotor.h")
    message(FATAL_ERROR "Unitree actuator SDK headers were not found under ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${UNITREE_ACTUATOR_SDK_LIBRARY}")
    message(FATAL_ERROR "Unitree actuator SDK library was not found at ${UNITREE_ACTUATOR_SDK_LIBRARY}")
endif()

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})

target_include_directories(junior_ctrl PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}
)

ament_target_dependencies(junior_ctrl
    rclcpp
    sensor_msgs
)

target_compile_options(junior_ctrl PRIVATE -O3)
target_link_libraries(junior_ctrl
    ${UNITREE_ACTUATOR_SDK_LIBRARY}
    pthread
)

install(TARGETS junior_ctrl
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/event2joy.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
