#include "ros/ros.h"
#include "std_msgs/String.h"

/*
    chatter topic에 새 메시지가 도착했을 때 호출되는 콜백 함수이다.
    메시지는 boost shared_ptr로 전달된다.
    즉, 원하는 경우 아래에서 삭제될 염려없이 기본 데이터를 복사하지 않고도 저장할 수 있다.
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  /*
      ROS Master와 함께 chatter topic을 subscribe한다.
      ROS는 새 메시지가 도착할 때마다 chatterCallback() 함수를 호출한다.

      두 번째 인수는 메시지를 충분히 빠르게 처리할 수 없는 queue의 크기이다.
      이 경우 큐가 1000개의 메시지에 도달했는데, 새 메시지가 도착하면
      이전 메시지를 버리기 시작한다.

      NodeHandle::subscribe()는 ros::Subscriber 객체를 반환하며,
      subscribe를 취소할 때까지 유지해야 한다.
      Subscriber 객체가 삭제되면 chatter topic에서 자동으로 subscribe이 취소된다.
  */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /*
      ros::spin()은 루프에 들어가 메시지 콜백을 최대한 빨리 호출한다.
      하지만 걱정할 필요가 없다.
      할 일이 없다면 CPU를 많이 사용하지 않는다.
      ros::spin()은 ros::ok()가 false를 반환하면 종료된다.
      이는 ros::shitdown()이 기본 Ctrl-C 핸들러에 의해 호출되거나,
      마스터가 종료를 지시하거나 수동으로 호출됨을 의미한다.
      이는 콜백을 펌핑하는 방법 중 하나이다.

      다시, 여기 Subscriber에서 무슨 일이 일어나고 있는지 요약하면 다음과 같다.

      1. ROS 시스템 초기화
      2. chatter topic을 구독(subscribe)
      3. Spin을 통해 메시지 도착을 대기한다.
      4. 메시지가 도착하면 등록한 chatterCallback() 함수가 호출된다.
  */
  ros::spin();

  return 0;
}