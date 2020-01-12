cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs)

add_message_files(
  FILES
  Pose2DStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs  # Or other packages containing msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aerial_robot_recognition_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
