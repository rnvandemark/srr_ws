cmake_minimum_required(VERSION 3.0.2)
project(srr_teleop)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# install all files in folders to their own shared folder
foreach(pkg_dir scripts)
  install(DIRECTORY
    "${pkg_dir}"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
  )
endforeach()
