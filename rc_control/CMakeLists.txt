cmake_minimum_required(VERSION 3.0.2)
project(rc_control)

## Catkin packages needed at build/run time
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ackermann_msgs
  geometry_msgs
)

catkin_package()

## Install Python nodes
catkin_install_python(
  PROGRAMS
    src/ackermann_cmd_mux.py
    src/throttle_interpolator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install non-code resources (configs, launches)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
