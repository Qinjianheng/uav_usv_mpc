cmake_minimum_required(VERSION 3.8)
project(uav_usv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerDiagnostic.msg"
  "msg/FollowTrajectory.msg"
  "msg/FollowPlanAck.msg"
  "msg/InterceptTrajectory.msg"
  "msg/InterceptResult.msg"
  "msg/MissionState.msg"
  "msg/PlannerDiagnostic.msg"
  "msg/PolynomialSegment.msg"
  "msg/PredictedTargetPoint.msg"
  "msg/TargetObservation.msg"
  "msg/TargetBearing.msg"
  "msg/TargetPrediction.msg"
  "msg/TargetState.msg"
  "msg/TrajectoryPoint.msg"
  "msg/UavState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
