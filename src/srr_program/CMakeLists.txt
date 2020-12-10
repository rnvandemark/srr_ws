cmake_minimum_required(VERSION 3.0.2)
project(srr_program)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs srr_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs srr_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(psm_vehicle
  src/VehicleProgramStateMachineNode.cpp
)
target_link_libraries(psm_vehicle
  ${catkin_LIBRARIES}
)

add_executable(psm_arm
  src/ArmProgramStateMachineNode.cpp
)
target_link_libraries(psm_arm
  ${catkin_LIBRARIES}
)

add_executable(psm_combined
  src/CombinedProgramStateMachineNode.cpp
)
target_link_libraries(psm_combined
  ${catkin_LIBRARIES}
)

install(TARGETS
  psm_vehicle
  psm_arm
  psm_combined
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
