cmake_minimum_required(VERSION 2.8.3)
project(surfel_next_best_view_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  DIRECTORY action
    FILES
    EvaluatePoses.action
    SetPointCloud.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surfel_next_best_view_msgs
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

