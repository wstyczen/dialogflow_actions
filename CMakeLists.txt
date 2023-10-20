cmake_minimum_required(VERSION 3.0.2)
project(dialogflow_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  pal_common_msgs
  rospy
  rospack
  sensor_msgs
  std_msgs
  tf
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES TurnToHuman.action MoveToHuman.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base_msgs nav_msgs pal_common_msgs rospy sensor_msgs std_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})
