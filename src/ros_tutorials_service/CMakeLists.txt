cmake_minimum_required(VERSION 3.0.2)
project(ros_tutorials_service)

## catkin 빌드를 할 때 요구되는 구성요소 패키지이다.
## 의존성 패키지로 message_generation, roscpp, std_msgs이며, 이 패키지들이 존재하지 않으면 빌드 도중에 에러가 난다.
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## 서비스 선언: SrvTutorial.srv
add_service_files(
  FILES
  SrvTutorial.srv
)

## 의존하는 메시지를 설정하는 옵션이다.
## std_msgs가 설치되어 있지 않다면 빌드 도중에 에러가 난다.
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin 패키지 옵션으로 라이브러리, catkin 빌드 의존성, 시스템 의존 패키지를 기술한다.
catkin_package(
  LIBRARIES ros_tutorials_service
  CATKIN_DEPENDS roscpp std_msgs
)

## 인클루드 디렉터리를 설정한다.
include_directories(${catkin_INCLUDE_DIRS})

## service_server 노드에 대한 빌드 옵션이다.
## 실행 파일, 타깃 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(service_server src/service_server.cpp)
add_dependencies(service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_server ${catkin_LIBRARIES})

## service_client 노드에 대한 빌드 옵션이다.
add_executable(service_client src/service_client.cpp)
add_dependencies(service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(service_client ${catkin_LIBRARIES})