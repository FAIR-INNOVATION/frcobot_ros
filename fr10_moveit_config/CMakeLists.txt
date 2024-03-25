cmake_minimum_required(VERSION 3.1.3)
project(fr10_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(gazebo_rh_pub src/gazebo_rh_pub.cpp)
add_dependencies(gazebo_rh_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_rh_pub ${catkin_LIBRARIES})

install(TARGETS gazebo_rh_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)