cmake_minimum_required(VERSION 3.0.2)
project(srr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(DIRECTORY msg FILES
  IndexedDoubleArr.msg
  RotationMatrix.msg
  TractionControlDebug.msg
  VehicleWheelRates.msg
)
add_service_files(DIRECTORY srv FILES
  CalculatePositionWithOffsets.srv
  GetDirectionOfRotation.srv
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
)
