#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"  // srv 파일에서 생성된 헤더 파일이다.

/*
    두 개의 int를 추가하는 서비스를 제공하며,
    srv 파일에 정의된 요청 및 응답 유형을 가져와서
    bool로 리턴한다.
*/
bool add(beginner_tutorials::AddTwoInts::Request  &req, 
    beginner_tutorials::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  // ROS에 add_two_ints라는 이름의 서비스를 등록한다.
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}