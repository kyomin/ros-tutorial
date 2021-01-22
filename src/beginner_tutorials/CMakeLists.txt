cmake_minimum_required(VERSION 3.0.2)

## 패키지 이름을 지정한다.
project(beginner_tutorials)



## catkin dependencies를 추적한다.
## 이것이 동작하기 위해서는 ros package 안에 include 폴더를 갖고 있어야 한다.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)



## ROS의 message를 등록한다.
## ros package 안에 msg 폴더를 추적한다.
add_message_files(
  DIRECTORY
  msg
  FILES
  Num.msg
)



## ROS의 service를 등록한다.
## ros package 안에 srv 폴더를 추적한다.
add_service_files(
  DIRECTORY
  srv
  FILES
  AddTwoInts.srv
)



## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)



## catkin 관련 빌드 정보 지정
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)



## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## C++로 작성된 ROS Node의 Topic을 위한 설정이다.
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)



## C++로 작성된 ROS Node의 Service를 위한 설정이다.
## 이렇게 하면 add_two_ints_server와 add_two_ints_client라는 두 개의 실행 파일이 생성된다.
## 기본적으로 ~/catkin_ws/devel/lib/<package name>에 있는 devel 공간의 패키지 디렉토리로 이동한다.
## 실행 파일을 직접 호출하거나, rosrun을 사용하여 호출할 수 있다.
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)