cmake_minimum_required(VERSION 3.0.2)
project(srr_utils)

add_compile_options(-std=c++14)

catkin_package(INCLUDE_DIRS include)

include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
