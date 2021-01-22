import rospy                    # ROS 파이썬을 위한 rospy 모듈 전체 가져오기.
from std_msgs.msg import String # std_msgs.msg 모듈 내에서 String만 콕 찍어 가져오기.

def talker():
    """
        ROS의 나머지 부분에 대한 talker의 인터페이스를 정의한다.
        talker 노드가 메시지 유형 String을 사용하여 chatter topic에 게시하고 있음을 선언한다.
        여기서 문자열은 실제로 std_msgs.msg.String 클래스이다.

        queue_size 인수는 ROS hydro의 New이며, 
        Subscriber가 충분히 빨리 수신하지 못하는 경우 
        대기중인 메시지의 양을 제한한다.
    """
    pub = rospy.Publisher('chatter', String, queue_size=10)

    """
        이는 rospy에게 노드의 이름을 알려주기 때문에 매우 중요한 문장이다.
        rospy가 이 정보를 가질 때까지는 ROS Master와 통신을 시작할 수 없다.
        이 경우 Node는 talker라는 이름을 취하게 된다.

        anonymous=True는 NAME 끝에 난수를 추가하여 노드가 고유한 이름을 갖도록 한다.
    """
    rospy.init_node('talker', anonymous=True)

    """
        인수 10을 사용하면 처리 시간이 1/10초를 통과하지 않는 한
        루프를 초당 10회 진행할 것으로 예상할 수 있다.
    """
    rate = rospy.Rate(10) # 10hz

    """
        이 루프는 상당히 표준적인 rospy 구조이다.
        rospy.is_shutdown() 플래그를 확인한 다음, 작업을 수행한다.
        프로그램이 종료되어야 하는지 확인하려면 is_shutdown()을 확인해야 한다(예: Ctrl-C 또는 기타).

        이 경우 작업은 chatter topic에 문자열을 게시하는
        pub.publish(hello_str)에 대한 호출이다.
        루프 내에서 rate.sleep()를 호출하는데,
        이는 루프를 통해 원하는 속도를 유지할 수 있을만큼 충분히 sleep한다.

        이 루프는 또한 rospy.loginfo(str)를 호출하여 삼중 작업을 수행한다.
        메시지가 화면에 인쇄되고 노드의 로그 파일에 기록되고, rosout에 기록된다.
        rosout은 디버깅을 위한 편리한 도구이다.
        노드의 출력이 있는 콘솔 창을 찾는 대신 rqt_console을 사용하여 메시지를 가져올 수 있다.
    """
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
