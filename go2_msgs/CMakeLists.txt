cmake_minimum_required(VERSION 3.8)
project(go2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConfirmedTarget.msg"
  "msg/DetectedHuman.msg"
  "msg/DetectedHumanArray.msg"
  "msg/SafetyAlert.msg"
  "msg/SafeVelocityCommand.msg"
  "msg/SafetyStatus.msg"
  "msg/BridgeStatus.msg"
  "msg/GroundingStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
