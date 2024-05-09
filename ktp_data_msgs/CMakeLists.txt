cmake_minimum_required(VERSION 3.8)
project(ktp_data_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "msg/StatusService.msg"
  "msg/StatusServiceEnv.msg"
  "msg/ServiceStatus.msg"
  "msg/ServiceStatusTask.msg"
  "msg/ServiceStatusTaskData.msg"
  "msg/ErrorReport.msg"
  "msg/Control.msg"
  "msg/ControlData.msg"
  "msg/ControlReport.msg"
  "msg/ControlReportData.msg"
  "msg/ControlReportDataGraphList.msg"
  "msg/Mission.msg"
  "msg/MissionTask.msg"
  "msg/MissionTaskData.msg"
  "msg/GraphList.msg"
  "msg/Graph.msg"
  "msg/GraphNodeList.msg"
  "msg/GraphEdgeList.msg"
  "msg/GraphZoneList.msg"
  "msg/GraphZoneVertices.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "msg/DetectedObjectSize.msg"
  "msg/ObstacleDetect.msg"
  "msg/LiDARSignal.msg"
  "srv/AssignControl.srv"
  "srv/AssignMission.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_generators)

ament_package()
