cmake_minimum_required(VERSION 3.0.2)
project(roscan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roar_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES roscan
  CATKIN_DEPENDS rospy roar_msgs sensor_msgs std_msgs
  #  DEPENDS system_lib
)
include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/roscan_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
