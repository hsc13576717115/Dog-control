cmake_minimum_required(VERSION 3.16)
project(custom_dog_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(CUSTOM_DOG_CONTROL_REAL_HARDWARE_DEFAULT ON)
else()
  set(CUSTOM_DOG_CONTROL_REAL_HARDWARE_DEFAULT OFF)
endif()

option(CUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE "Build the GO-M8010-6 ros2_control hardware plugin"
       ${CUSTOM_DOG_CONTROL_REAL_HARDWARE_DEFAULT})
option(CUSTOM_DOG_CONTROL_BUILD_TESTING "Build custom_dog_control unit tests" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono filesystem log log_setup regex system thread)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

# Some Humble Pinocchio/urdfdom exports use bare -l names for multi-arch
# libraries. Keep those exports usable on both x86_64 and ARM64 installations.
set(CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)
set(CUSTOM_DOG_CONTROL_PREFIXES ${CMAKE_PREFIX_PATH})
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" CUSTOM_DOG_CONTROL_AMENT_PREFIXES
    "$ENV{AMENT_PREFIX_PATH}")
  list(APPEND CUSTOM_DOG_CONTROL_PREFIXES ${CUSTOM_DOG_CONTROL_AMENT_PREFIXES})
endif()
if(DEFINED ENV{ROS_DISTRO})
  list(APPEND CUSTOM_DOG_CONTROL_PREFIXES "/opt/ros/$ENV{ROS_DISTRO}")
endif()
list(REMOVE_DUPLICATES CUSTOM_DOG_CONTROL_PREFIXES)
foreach(prefix IN LISTS CUSTOM_DOG_CONTROL_PREFIXES)
  if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS
      "${prefix}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
  endif()
  foreach(architecture_dir IN ITEMS
      "${prefix}/lib/x86_64-linux-gnu"
      "${prefix}/lib/aarch64-linux-gnu"
      "${prefix}/lib/arm64-linux-gnu")
    if(IS_DIRECTORY "${architecture_dir}")
      list(APPEND CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS "${architecture_dir}")
    endif()
  endforeach()
endforeach()
list(REMOVE_DUPLICATES CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)

# Some ROS 2 Humble exported targets use bare -l names for urdfdom. Make the
# discovered multi-arch directories visible to both package targets and the
# ament test executables.
if(CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)
  link_directories(${CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS})
endif()

# qpOASES is pinned as a Git submodule. Building the source list here avoids its
# legacy project() call changing this package's compiler flags.
file(GLOB QPOASES_SOURCES CONFIGURE_DEPENDS third_party/qpOASES/src/*.cpp)
add_library(custom_dog_control_qpoases STATIC ${QPOASES_SOURCES})
set_target_properties(custom_dog_control_qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(custom_dog_control_qpoases SYSTEM PUBLIC third_party/qpOASES/include)
target_compile_definitions(custom_dog_control_qpoases PRIVATE __NO_COPYRIGHT__ LINUX)

add_library(custom_dog_control_wbc STATIC
  third_party/legged_wbc/src/HoQp.cpp
  third_party/legged_wbc/src/WbcBase.cpp
  third_party/legged_wbc/src/HierarchicalWbc.cpp
  third_party/legged_wbc/src/WeightedWbc.cpp
)
set_target_properties(custom_dog_control_wbc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(custom_dog_control_wbc PUBLIC third_party/legged_wbc/include)
ament_target_dependencies(custom_dog_control_wbc
  ocs2_centroidal_model
  ocs2_core
  ocs2_legged_robot
  ocs2_pinocchio_interface
  pinocchio
)
if(CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)
  target_link_directories(custom_dog_control_wbc PUBLIC
    ${CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS})
endif()
target_link_libraries(custom_dog_control_wbc custom_dog_control_qpoases Eigen3::Eigen)

# ROS-free algorithm sources vendored from qiayuanl/legged_control. Keeping
# this as a separate target makes the runtime backend and its provenance
# explicit instead of silently substituting the OCS2 example interface.
add_library(custom_dog_control_legged_interface STATIC
  third_party/legged_control/legged_interface/src/constraint/EndEffectorLinearConstraint.cpp
  third_party/legged_control/legged_interface/src/constraint/FrictionConeConstraint.cpp
  third_party/legged_control/legged_interface/src/constraint/ZeroForceConstraint.cpp
  third_party/legged_control/legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp
  third_party/legged_control/legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp
  third_party/legged_control/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp
  third_party/legged_control/legged_interface/src/initialization/LeggedRobotInitializer.cpp
  third_party/legged_control/legged_interface/src/SwitchedModelReferenceManager.cpp
  third_party/legged_control/legged_interface/src/LeggedRobotPreComputation.cpp
  third_party/legged_control/legged_interface/src/LeggedInterface.cpp
)
set_target_properties(custom_dog_control_legged_interface PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(custom_dog_control_legged_interface PUBLIC
  third_party/legged_control/legged_interface/include)
ament_target_dependencies(custom_dog_control_legged_interface
  ocs2_centroidal_model
  ocs2_core
  ocs2_legged_robot
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_self_collision
  ocs2_sqp
  pinocchio
)
if(CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)
  target_link_directories(custom_dog_control_legged_interface PUBLIC
    ${CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS})
endif()
target_link_libraries(custom_dog_control_legged_interface Eigen3::Eigen)

add_library(custom_dog_control_core STATIC
  src/nmpc/ModelValidator.cpp
  src/nmpc/KinematicStateEstimator.cpp
  src/nmpc/NmpcBackend.cpp
  src/safety/SafetyMonitor.cpp
)
set_target_properties(custom_dog_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(custom_dog_control_core PUBLIC include)
if(CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS)
  target_link_directories(custom_dog_control_core PUBLIC
    ${CUSTOM_DOG_CONTROL_MULTIARCH_LIB_DIRS})
endif()
ament_target_dependencies(custom_dog_control_core
  ament_index_cpp
  ocs2_centroidal_model
  ocs2_core
  ocs2_legged_robot
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_self_collision
  ocs2_sqp
  pinocchio
  urdf
)
target_link_libraries(custom_dog_control_core
  custom_dog_control_legged_interface
  custom_dog_control_wbc
  Eigen3::Eigen
  yaml-cpp)

add_library(custom_dog_control_nmpc_wbc_controller SHARED
  src/controller/NmpcWbcController.cpp
)
target_include_directories(custom_dog_control_nmpc_wbc_controller PUBLIC include)
ament_target_dependencies(custom_dog_control_nmpc_wbc_controller
  controller_interface
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
)
target_link_libraries(custom_dog_control_nmpc_wbc_controller custom_dog_control_core)

if(CUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE)
  set(UNITREE_ACTUATOR_SDK_ROOT "$ENV{UNITREE_ACTUATOR_SDK_ROOT}" CACHE PATH
      "Root of the Unitree actuator SDK")
  if(NOT UNITREE_ACTUATOR_SDK_ROOT)
    message(FATAL_ERROR
      "CUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON requires UNITREE_ACTUATOR_SDK_ROOT. "
      "Pass -DUNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk")
  endif()

  find_path(UNITREE_ACTUATOR_SDK_INCLUDE_DIR
    NAMES unitreeMotor/unitreeMotor.h
    HINTS "${UNITREE_ACTUATOR_SDK_ROOT}/include"
    NO_DEFAULT_PATH)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(UNITREE_ACTUATOR_SDK_LIBRARY_NAMES
      UnitreeMotorSDK_Arm64 UnitreeMotorSDK_arm64)
  else()
    set(UNITREE_ACTUATOR_SDK_LIBRARY_NAMES
      UnitreeMotorSDK_Linux64 UnitreeMotorSDK_amd64)
  endif()
  find_library(UNITREE_ACTUATOR_SDK_LIBRARY
    NAMES ${UNITREE_ACTUATOR_SDK_LIBRARY_NAMES}
    HINTS "${UNITREE_ACTUATOR_SDK_ROOT}/lib"
    NO_DEFAULT_PATH)
  if(NOT UNITREE_ACTUATOR_SDK_INCLUDE_DIR OR NOT UNITREE_ACTUATOR_SDK_LIBRARY)
    message(FATAL_ERROR
      "Unitree actuator SDK headers or a compatible ${CMAKE_SYSTEM_PROCESSOR} "
      "library were not found under ${UNITREE_ACTUATOR_SDK_ROOT}")
  endif()

  add_library(custom_dog_control_unitree_system SHARED
    src/hardware/UnitreeSystemInterface.cpp
    src/hardware/IOSDK.cpp
  )
  target_compile_definitions(custom_dog_control_unitree_system PRIVATE COMPILE_WITH_REAL_ROBOT)
  target_include_directories(custom_dog_control_unitree_system PUBLIC
    include
    ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}
  )
  ament_target_dependencies(custom_dog_control_unitree_system
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(custom_dog_control_unitree_system
    ${UNITREE_ACTUATOR_SDK_LIBRARY}
    pthread
  )
  set_target_properties(custom_dog_control_unitree_system PROPERTIES
    INSTALL_RPATH "${UNITREE_ACTUATOR_SDK_ROOT}/lib"
  )
  pluginlib_export_plugin_description_file(hardware_interface custom_dog_control_hardware_plugins.xml)
  install(TARGETS custom_dog_control_unitree_system LIBRARY DESTINATION lib)
endif()

pluginlib_export_plugin_description_file(controller_interface custom_dog_control_controller_plugins.xml)

install(TARGETS
  custom_dog_control_qpoases
  custom_dog_control_legged_interface
  custom_dog_control_wbc
  custom_dog_control_core
  custom_dog_control_nmpc_wbc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/custom_dog_control DESTINATION include)
install(DIRECTORY config/nmpc DESTINATION share/${PROJECT_NAME}/config)
install(FILES
  config/controllers.yaml
  config/dependencies.lock.yaml
  config/legged_control_upstream.sha256
  config/real_controller.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES
  launch/gazebo.launch.py
  launch/real.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY rviz worlds DESTINATION share/${PROJECT_NAME})
install(FILES urdf/custom_dog.ros2_control.xacro DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES custom_dog_control_controller_plugins.xml DESTINATION share/${PROJECT_NAME})
install(FILES
  third_party/legged_control/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME legged_control.BSD-3-Clause.txt
)
install(FILES
  third_party/legged_wbc/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME legged_wbc.BSD-3-Clause.txt
)
install(FILES
  third_party/qpOASES/LICENSE.txt
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME qpOASES.LGPL-2.1.txt
)
if(CUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE)
  install(FILES custom_dog_control_hardware_plugins.xml DESTINATION share/${PROJECT_NAME})
endif()
install(PROGRAMS
  scripts/check_dependencies.sh
  scripts/build_simulation.sh
  scripts/event2joy.py
  scripts/fetch_ocs2.sh
  scripts/keyboard_teleop.py
  scripts/profile_runtime.py
  scripts/simulation_command_matrix_test.py
  scripts/simulation_envelope_test.py
  scripts/simulation_motion_test.py
  scripts/simulation_smoke_test.py
  scripts/simulation_terrain_test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY patches repos DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING AND CUSTOM_DOG_CONTROL_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_legged_control_provenance
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/check_legged_control_provenance.py
            ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_gtest(test_control_contract test/test_control_contract.cpp)
  target_include_directories(test_control_contract PRIVATE include)
  target_link_libraries(test_control_contract Eigen3::Eigen)

  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp)
  target_include_directories(test_safety_monitor PRIVATE include)
  target_link_libraries(test_safety_monitor custom_dog_control_core)

  set(CUSTOM_DOG_CONTROL_CANONICAL_URDF
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../ros2/src/custom_dog_description/urdf/custom_dog.urdf"
      CACHE FILEPATH "Canonical custom dog URDF used by the model contract test")
  if(EXISTS "${CUSTOM_DOG_CONTROL_CANONICAL_URDF}")
    add_test(
      NAME test_urdf_contract
      COMMAND ${Python3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/test/check_urdf_contract.py
              ${CUSTOM_DOG_CONTROL_CANONICAL_URDF}
    )
    ament_add_gtest(test_model_validator test/test_model_validator.cpp)
    target_include_directories(test_model_validator PRIVATE include)
    target_compile_definitions(
      test_model_validator PRIVATE
      CUSTOM_DOG_CONTROL_CANONICAL_URDF="${CUSTOM_DOG_CONTROL_CANONICAL_URDF}")
    target_link_libraries(test_model_validator custom_dog_control_core)
  else()
    message(WARNING "Skipping URDF contract test: ${CUSTOM_DOG_CONTROL_CANONICAL_URDF} does not exist")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  custom_dog_control_core
  custom_dog_control_legged_interface
  custom_dog_control_wbc)
ament_export_dependencies(controller_interface hardware_interface rclcpp)
ament_package()
