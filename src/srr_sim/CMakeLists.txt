cmake_minimum_required(VERSION 3.0.2)
project(srr_sim)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

function(compile_urdf filename)
  if(DEFINED ARGV0)
    MESSAGE(STATUS "Running xacro (${ARGV0})...")
    execute_process (
      COMMAND bash -c "pushd ${CMAKE_CURRENT_SOURCE_DIR}/urdf > /dev/null; rosrun xacro xacro -o ${ARGV0} ${ARGV0}.xacro; popd > /dev/null;"
      OUTPUT_VARIABLE URDF_OUTPUT
    )
	  MESSAGE(STATUS "${URDF_OUTPUT}")
  else()
    message(FATAL_ERROR "URDF filename not specified but is required.")
  endif()
endfunction(compile_urdf)

compile_urdf(srr_integrated.urdf)
compile_urdf(sra_integrated.urdf)

# install all files in folders to their own shared folder
foreach(pkg_dir world launch config meshes urdf)
  install(DIRECTORY
    "${pkg_dir}"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
  )
endforeach()
