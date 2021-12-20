cmake_minimum_required(VERSION 3.5)
project(lauv_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch meshes robots urdf
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(xacro REQUIRED)

  foreach(T
      test/test_urdf_files.py
      )
    add_launch_test(${T})
  endforeach()
endif()

ament_package()
