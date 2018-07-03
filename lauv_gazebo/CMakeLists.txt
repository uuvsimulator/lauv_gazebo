cmake_minimum_required(VERSION 2.8.3)
project(lauv_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
