cmake_minimum_required(VERSION 3.0.2)
project(startupday_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES startupday_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/demo.py
  src/transform_pose_marker.py
  src/get_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
