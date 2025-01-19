import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CentralMonitor(Node):
    def __init__(self):
        super().__init__('central_monitor')

        # 활성 로봇 토픽 및 서브스크라이버 관리
        self.active_topics = set()  # 현재 활성화된 로봇 토픽 저장
        self.topic_subscriptions = {}  # 토픽 이름과 서브스크라이버 매핑 (subscriptions → topic_subscriptions)

        # 주기적으로 활성 토픽 업데이트
        self.timer = self.create_timer(2.0, self.update_subscriptions)

    def update_subscriptions(self):
        """
        ROS 2 네트워크에서 활성화된 토픽을 탐색하고,
        새로운 로봇 토픽이 발견되면 서브스크라이버를 추가합니다.
        """
        topics = self.get_topic_names_and_types()

        # 'robot_<id>_state' 형식의 토픽만 필터링
        robot_topics = [t[0] for t in topics if 'robot_' in t[0] and '_state' in t[0]]

        # 새로운 토픽 감지 및 구독
        for topic in robot_topics:
            if topic not in self.active_topics:
                self.get_logger().info(f'New robot detected: {topic}')
                self.topic_subscriptions[topic] = self.create_subscription(  # 변경된 변수명 사용
                    String,
                    topic,
                    self.listener_callback,
                    10)
                self.active_topics.add(topic)

        # 중단된 토픽 제거
        inactive_topics = self.active_topics - set(robot_topics)
        for topic in inactive_topics:
            self.get_logger().info(f'Robot disconnected: {topic}')
            del self.topic_subscriptions[topic]  # 변경된 변수명 사용
            self.active_topics.remove(topic)

    def listener_callback(self, msg):
        """
        각 로봇의 상태 메시지를 처리합니다.
        배터리가 20% 이하인 경우 경고를 출력합니다.
        """
        self.get_logger().info(f'Monitoring: {msg.data}')
        if "Battery" in msg.data:
            try:
                # 배터리 상태 파싱
                battery_level = int(msg.data.split("Battery ")[1].split("%")[0])
                if battery_level <= 20:
                    self.get_logger().warning(f'Low battery warning! {msg.data}')
            except (IndexError, ValueError):
                self.get_logger().error(f'Failed to parse battery level: {msg.data}')
        else:
            self.get_logger().info(f'No battery information in message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CentralMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
