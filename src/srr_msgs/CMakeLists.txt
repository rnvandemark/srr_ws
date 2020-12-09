cmake_minimum_required(VERSION 3.0.2)
project(srr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(DIRECTORY msg FILES
  IndexedDoubleArr.msg
  Point2D.msg
  RotationMatrix.msg
  TractionControlDebug.msg
  VehicleWheelRates.msg
  VehicleVelKinSegment.msg
  VehiclePath.msg
  VehicleVelKinSolution.msg
  VehicleVelKinDebug.msg
  ArmJointPose.msg
  ArmCartesianPose.msg
)
add_service_files(DIRECTORY srv FILES
  CalculatePositionWithOffsets.srv
  GetDirectionOfRotation.srv
  CalculateVehicleVelKin.srv
  CalculateArmFwdKin.srv
  CalculateArmInvKin.srv
  CalculateCombinedKin.srv
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  geometry_msgs
)
