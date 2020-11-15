cmake_minimum_required(VERSION 3.0.2)
project(srr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_service_files(DIRECTORY srv FILES
  CalculatePositionWithOffsets.srv
)
generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
