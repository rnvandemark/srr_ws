cmake_minimum_required(VERSION 3.0.2)
project(srr_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

MESSAGE(STATUS "Running xacro...")
execute_process (
  COMMAND bash -c "pushd ${CMAKE_CURRENT_SOURCE_DIR}/urdf > /dev/null; rosrun xacro xacro -o srr_integrated.urdf srr_integrated.urdf.xacro > /dev/null; popd > /dev/null;"
  OUTPUT_VARIABLE IGNORED_OUTPUT
)

# install all files in folders to their own shared folder
foreach(pkg_dir world launch config meshes urdf)
  install(DIRECTORY
    "${pkg_dir}"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
  )
endforeach()
