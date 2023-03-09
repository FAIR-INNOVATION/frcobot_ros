cmake_minimum_required(VERSION 3.0.2)
project(frcobot_gripper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frcobot_gripper
#  CATKIN_DEPENDS roscpp rospy rviz std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/frcobot_gripper
  include/xmlrpc-c
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES
  include/frcobot_gripper/gripper_io_pad.h
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  ## Here we specify which header files need to be run through "moc",
  ## Qt's meta-object compiler.
  qt5_wrap_cpp(MOC_FILES
  include/frcobot_gripper/gripper_io_pad.h
  )
endif()

add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/gripper_io_pad.cpp 
  ${MOC_FILES}
)

set(TEST_FILES
  test/gripper_plugin_test.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME} ${SOURCE_FILES})
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/frcobot_gripper.cpp
# )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
  robotAPI
  xmlrpcpp
)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_executable(gripper_plugin_test ${TEST_FILES})
target_link_libraries(gripper_plugin_test ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
  robotAPI
  # xmlrpcpp
  xmlrpc
  xmlrpc_client
  xmlrpc_util
  xmlrpc_xmlparse
  xmlrpc_xmltok
  z
)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/gripper_plugin_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
