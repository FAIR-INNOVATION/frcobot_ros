cmake_minimum_required(VERSION 3.0.2)
project(frcobot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY meshes
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY urdf
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

include_directories(
  ${catkin_INCLUDE_DIRS}
)