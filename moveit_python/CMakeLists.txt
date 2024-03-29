cmake_minimum_required(VERSION 3.0.2)
project(moveit_python)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  shape_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
catkin_python_setup()

catkin_install_python(
  PROGRAMS
    scripts/add_box.py
    scripts/attach_box.py
    scripts/dump_planning_scene.py
    scripts/list_objects.py
    scripts/load_planning_scene.py
    scripts/remove_objects.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
