include_directories(
  include/
)

# FrRobot Hardware Interface
add_library(frrobot_hw_interface
  src/frrobot_hw_interface.cpp
  src/tool.cpp
)
target_link_libraries(frrobot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(frrobot_hw_main src/frrobot_hw_main.cpp)
target_link_libraries(frrobot_hw_main
  frrobot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    frrobot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    frrobot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/frrobot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
