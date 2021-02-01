cmake_minimum_required(VERSION 3.0.2)
project(robot_gui_bridge)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui_bridge
#  CATKIN_DEPENDS rosbridge_server
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)