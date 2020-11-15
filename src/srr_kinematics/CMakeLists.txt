cmake_minimum_required(VERSION 3.0.2)
project(srr_kinematics)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS roscpp srr_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp srr_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(incorporate_joints_offsets_server
  src/IncorporateJointOffsetsServer.cpp
  src/JointOffsetsCollection.cpp
)
target_link_libraries(incorporate_joints_offsets_server
  ${catkin_LIBRARIES}
)
install(TARGETS incorporate_joints_offsets_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
