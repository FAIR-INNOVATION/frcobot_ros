cmake_minimum_required(VERSION 3.0.2)
project(frcobot_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  include/frcobot_camera
  include/xmlrpc-c
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

set(EYEINHAND src/eyeinhand.cpp)

set(BINPICKING src/binPicking.cpp)

add_executable(binpicking  ${BINPICKING})
target_link_libraries(binpicking  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/librobotAPI.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_client.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_util.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_xmlparse.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_xmltok.so
  ${PROJECT_SOURCE_DIR}/lib/libz.so
)

add_executable(eyeinhand  ${EYEINHAND})
target_link_libraries(eyeinhand  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/librobotAPI.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_client.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_util.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_xmlparse.so
  ${PROJECT_SOURCE_DIR}/lib/libxmlrpc_xmltok.so
  ${PROJECT_SOURCE_DIR}/lib/libz.so
)