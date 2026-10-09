cmake_minimum_required(VERSION 3.16)
project(qr_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContact.msg"
  "msg/FootContactArray.msg"
  "msg/RobotState.msg"
  "msg/SupportRegion.msg"
  "msg/SupportRegionArray.msg"
  "msg/Footstep.msg"
  "msg/FootstepPlan.msg"
  "msg/ExecutionStatus.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/MissionStatus.msg"
  "srv/PlanFootsteps.srv" "action/ExecuteFootsteps.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
