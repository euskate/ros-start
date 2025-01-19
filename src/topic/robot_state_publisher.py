import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sys
import random

class RobotStatePublisher(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}_publisher')

        # QoS 설정: 신뢰성(RELIABLE), 지속성(VOLATILE)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.publisher_ = self.create_publisher(String, f'robot_{robot_id}_state', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_state)  # 1초마다 상태 발행
        self.charge_timer = self.create_timer(30.0, self.charge_battery)  # 30초마다 배터리 충전

        self.robot_id = robot_id
        self.battery = 100  # 초기 배터리
        self.speed = 1.5  # 초기 속도

    def publish_state(self):
        """로봇 상태를 발행"""
        msg = String()

        if self.battery > 0:
            # 랜덤 속도 변화
            self.speed = random.uniform(0.5, 2.0)

            # 배터리 상태 메시지
            msg.data = f'Robot {self.robot_id}: Battery {self.battery}%, Speed {self.speed:.2f} m/s'
            self.publisher_.publish(msg)

            # 배터리 감소
            self.battery -= 1
        else:
            # 배터리가 없을 때 메시지
            self.speed = 0
            msg.data = f'Robot {self.robot_id}: Battery depleted, Speed {self.speed} m/s'
            self.publisher_.publish(msg)

        self.get_logger().info(f'Published: {msg.data}')

    def charge_battery(self):
        """배터리 충전"""
        if self.battery < 100:
            self.battery += 10
            if self.battery > 100:
                self.battery = 100  # 최대 배터리 제한
            self.get_logger().info(f'Battery charging... Current battery: {self.battery}%')

def main(args=None):
    rclpy.init(args=args)

    # 명령줄 인자로 로봇 ID 받기
    if len(sys.argv) < 2:
        print("Usage: python3 robot_state_publisher.py <robot_id>")
        return

    robot_id = sys.argv[1]
    node = RobotStatePublisher(robot_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
