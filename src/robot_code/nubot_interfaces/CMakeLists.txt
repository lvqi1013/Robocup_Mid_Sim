cmake_minimum_required(VERSION 3.8)
project(nubot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/ActionCmd.msg"
  "msg/Angle.msg"
  "msg/BallInfo.msg"
  "msg/BallInfo3D.msg"
  "msg/BallIsHolding.msg"
  "msg/CoachInfo.msg"
  "msg/CoachWorldModelInfo.msg"
  "msg/CurrentCmd.msg"
  "msg/MotionCmd.msg"
  "msg/MotorInfo.msg"
  "msg/ObjectInfo.msg"
  "msg/ObstaclesInfo.msg"
  "msg/ObstaclesInfo3D.msg"
  "msg/OdoInfo.msg"
  "msg/OminiVisionInfo.msg"
  "msg/PassCommands.msg"
  "msg/Point3D.msg"
  "msg/PPoint.msg"
  "msg/Relocation.msg"
  "msg/RobotInfo.msg"
  "msg/SendingOff.msg"
  "msg/SimulationStrategy.msg"
  "msg/StrategyInfo.msg"
  "msg/TargetInfo.msg"
  "msg/TeleopJoy.msg"
  "msg/VelCmd.msg"
  "msg/WorldModelInfo.msg"
  "srv/DribbleId.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(${PROJECT_NAME}_include INTERFACE)

target_include_directories(${PROJECT_NAME}_include INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}_include
  EXPORT export_${PROJECT_NAME}_include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME}_include  HAS_LIBRARY_TARGET)
ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
