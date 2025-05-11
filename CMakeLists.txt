cmake_minimum_required(VERSION 3.0.2)
project(pickasso_navigation_stack)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  auto_mapping/scripts/auto_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)