cmake_minimum_required(VERSION 2.8.3)
project(lauv_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T
      test/test_urdf_files.test
      )
     add_rostest(${T})
  endforeach()
endif()