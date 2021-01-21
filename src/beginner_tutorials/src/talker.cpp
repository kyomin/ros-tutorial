// ros/ros.h는 ROS 시스템의 가장 일반적인 공용 부분을 사용하는 데 필요한 모든 헤더를 포함하는 편의 기능이다.
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
 
int main(int argc, char **argv) {
    /*
        ROS를 초기화한다.
        이를 통해 ROS는 명령 줄을 통해 이름을 다시 매핑할 수 있다.
        노드의 이름을 지정하는 부분이라 생각한다.
        노드 이름은 실행중인 시스템에서 고유해야 한다.
    */
    ros::init(argc, argv, "talker");

    /*
        이 프로세스의 노드에 대한 핸들을 만든다.
        생성 된 첫 번째 NodeHandle은 실제로 노드의 초기화를 수행하고,
        마지막으로 파괴 된 노드는 노드가 사용하던 모든 리소스를 정리한다.
    */
    ros::NodeHandle n;

    /*
        topic chatter에 std_msgs/String 유형의 메시지를 게시(publish) 할 것이라고 ROS 마스터에게 알린다.
        이를 통해 마스터는 채팅을 듣고있는 모든 노드에게 해당 주제(topic)에 대한 데이터를 게시할 것임을 알릴 수 있다.

        두 번째 인수는 게시(publishing) 대기열(queue)의 크기이다.
        이 경우, 너무 빨리 publish하는 경우 이번 메시지를 버리기 시작하기 전에 최대 1000개의 메시지를 버퍼링한다.

        NodeHandle::advertise()는 두 가지 용도로 사용되는 ros::Publisher 객체를 반환한다.
        1) 생성 된 topic에 메시지를 게시할 수 있는 publish() 메소드를 포함.
        2) 범위를 벗어날 때, 자동으로 advertise가 취소된다.
    */
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    /*
        ros::Rate 객체를 사용하면 반복할 주파수를 지정할 수 있다.
        Rate::sleep()에 대한 마지막 호출 이후 얼마나 지났는지 추적하고 정확학 시간 동안 잠든다.

        이 경우 우리는 10Hz에서 실행하고 싶다고 말한다.
    */
    ros::Rate loop_rate(10);

    /*
        기본적으로 roscpp는 Ctrl-C 처리를 제공하는 SIGINT 핸들러를 설치하여 ros::ok()가 false를 반환하도록 한다.
        ros::ok()는 다음과 같은 경우 false를 반환한다.

        1. SIGINT 수신 (Ctrl-C)
        2. 같은 이름을 가진 다른 노드에 의해 네트워크에서 킥오프 됨
        3. ros::shutdown()이 응용 프로그램의 다른 부분에서 호출
        4. 모든 ros::NodeHandles가 파되

        ros::ok()가 false를 반환하면 모든 ROS 호출이 실패한다.
    */
    int count = 0;
    while (ros::ok()) {
        /*
            일반적으로 msg 파일에서 생성되는 message-adapted 클래스를 사용하여, ROS에서 메시지를 브로드캐스트 한다.
            더 복잡한 데이터 유형이 가능하지만, 지금은 "data"라는 하나의 멤버가 있는 표준 문자열 메시지를 사용한다.
        */
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        /*
            이제 우리는 실제로 연결된 모든 사람에게 메시지를 방송(breoadcast)한다.
        */
        chatter_pub.publish(msg);

        ROS_INFO("%s", msg.data.c_str());

        /*
            콜백을 수신하지 않기 때문에 여기에선 ros::spinOnce()를 호출할 필요가 없다.
            그러나 이 애플리케이션에 구독(subscription)을 추가한다면 이 함수 호출이 필요하다.
            호출하지 않으면 콜백이 호출되지 않기 때문이다.
        */
        ros::spinOnce();

        /*
            이제 ros::Rate 객체를 사용하여 10Hz 게시(publish) 속도에 도달할 수 있도록 남은 시간 동안 휴면한다.
            다음은 무슨 일이 일어나는지 요약한 것이다.

            1. ROS 시스템 초기화
            2. chatter topic에 대한 std_msgs/String 메시지를 publish할 것이라고 마스터에게 알린다.
            3. 1초에 10번 chatter에 메시지를 publish하는 동안 반복

            이제 메시지를 수신하는 노드를 작성해야 한다.
        */
        loop_rate.sleep();
        ++count;
    }

    return 0;
}