cmake_minimum_required(VERSION 3.0.2)
project(srr_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

# install all files in folders to their own shared folder
foreach(pkg_dir world launch)
  install(DIRECTORY
    "${pkg_dir}"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
  )
endforeach()
