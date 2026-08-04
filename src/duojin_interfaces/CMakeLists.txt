cmake_minimum_required(VERSION 3.8)
project(duojin_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmMotionStatus.msg"
  "action/MoveArmPose.action"
  "action/MoveArmRelative.action"
  "action/MoveArmJoints.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
