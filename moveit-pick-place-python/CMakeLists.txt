cmake_minimum_required(VERSION 3.0.2)
project(pick_place_python)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  shape_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
