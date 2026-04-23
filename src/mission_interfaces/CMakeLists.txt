cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CircleMission.action"
  "srv/RelayHandoff.srv"
  "msg/GlobalMission.msg"
  "msg/RelayStatus.msg"
)
ament_package()
