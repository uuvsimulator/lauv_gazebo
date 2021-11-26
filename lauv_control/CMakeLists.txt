cmake_minimum_required(VERSION 3.5)
project(lauv_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)

ament_package()
