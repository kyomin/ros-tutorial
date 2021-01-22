#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3) {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;

  /*
    이를 통해 'add_two_ints' 서비스에 대한 클라이언트가 생성된다.
    ros::ServiceClient 객체는 나중에 서비스를 호출하는 데 사용된다.
  */
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  
  /*
    여기에서 자동 생성 된 서비스 클래스를 인스턴스화 하고,
    값을 요청 멤버에 할당한다.

    서비스 클래스에는 요청(Request)과 응답(Response)의 두 멤버가 포함된다.
    또한, 이 두 개의 클래스(Request, Response)정의도 갖고 있다.
  */
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  /*
    client.call(srv)를 통해 실제로 서비스를 호출한다.
    서비스 호출은 blocking 되므로, 호출이 완료되면 반환된다.

    서비스 호출이 성공하면 call()은 true를 반환하고,
    srv.response의 값은 유효하다.

    호출이 성공하지 못하면 call()은 false를 반환하고,
    srv.response의 값은 유효하지 않다.
  */
  if (client.call(srv)) {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}