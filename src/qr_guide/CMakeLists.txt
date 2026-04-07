cmake_minimum_required(VERSION 3.14)
project(qr_guide)

set(PLATFORM arm64)
set(SIMULATION OFF)
set(REAL_ROBOT ON)
set(DEBUG OFF)
set(MOVE_BASE OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()
if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
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
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(UNITREE_ACTUATOR_SDK_ROOT "/home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main" CACHE PATH "Local Unitree actuator SDK root")
set(UNITREE_ACTUATOR_SDK_INCLUDE_DIR "${UNITREE_ACTUATOR_SDK_ROOT}/include")
set(UNITREE_ACTUATOR_SDK_LIBRARY "${UNITREE_ACTUATOR_SDK_ROOT}/lib/libUnitreeMotorSDK_Arm64.so")

if(NOT EXISTS "${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}/unitreeMotor/unitreeMotor.h")
    message(FATAL_ERROR "Unitree actuator SDK headers were not found under ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${UNITREE_ACTUATOR_SDK_LIBRARY}")
    message(FATAL_ERROR "Unitree actuator SDK library was not found at ${UNITREE_ACTUATOR_SDK_LIBRARY}")
endif()

set(QR_GUIDE_CORE_SOURCES
    src/config/RobotConfig.cpp
    src/model/LegKinematics.cpp
    src/model/RobotModel.cpp
    src/input/JoystickMapper.cpp
    src/runtime/ControllerNode.cpp
    src/runtime/RobotRunner.cpp
    src/runtime/VisualizationPublisher.cpp
    src/common/LowPassFilter.cpp
    src/common/unitreeRobot.cpp
    src/control/Estimator.cpp
    src/FSM/FSM.cpp
    src/FSM/FSMState.cpp
    src/FSM/State_Passive.cpp
    src/FSM/State_FixedStand.cpp
    src/FSM/State_Trotting.cpp
    src/FSM/State_StepTest.cpp
    src/interface/IOSDK.cpp
    src/quadProgpp/Array.cc
    src/quadProgpp/QuadProg++.cc
)

add_library(qr_guide_core ${QR_GUIDE_CORE_SOURCES})
target_include_directories(qr_guide_core PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}
)
ament_target_dependencies(qr_guide_core
    ament_index_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    visualization_msgs
)
target_compile_options(qr_guide_core PRIVATE -O3)
target_link_libraries(qr_guide_core
    yaml-cpp
    ${UNITREE_ACTUATOR_SDK_LIBRARY}
    pthread
)

add_executable(junior_ctrl src/main.cpp)
target_include_directories(junior_ctrl PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}
)
ament_target_dependencies(junior_ctrl
    ament_index_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    visualization_msgs
)
target_compile_options(junior_ctrl PRIVATE -O3)
target_link_libraries(junior_ctrl
    qr_guide_core
    yaml-cpp
)

install(TARGETS qr_guide_core junior_ctrl
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/event2joy.py
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    launch/dog.launch.py
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
