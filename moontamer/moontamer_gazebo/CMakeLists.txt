cmake_minimum_required(VERSION 2.8.3)
project(moontamer_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
