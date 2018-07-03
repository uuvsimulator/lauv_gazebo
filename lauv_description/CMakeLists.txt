cmake_minimum_required(VERSION 2.8.3)
project(lauv_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
