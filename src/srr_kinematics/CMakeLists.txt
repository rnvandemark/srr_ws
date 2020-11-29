cmake_minimum_required(VERSION 3.0.2)
project(srr_kinematics)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS roscpp srr_msgs srr_utils)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp srr_msgs sensor_msgs gazebo_msgs srr_utils
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

add_executable(traction_controller
  src/TractionController.cpp
  src/TractionControlContainer.cpp
)
target_link_libraries(traction_controller
  ${catkin_LIBRARIES}
)

install(TARGETS
  incorporate_joints_offsets_server
  traction_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
