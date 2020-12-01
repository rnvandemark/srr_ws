cmake_minimum_required(VERSION 3.0.2)
project(srr_utils)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS srr_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS srr_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
