import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import random
import math

class TurtleRace(Node):
    def __init__(self):
        super().__init__('turtle_race')
        self.turtle_names = ['turtle1', 'turtle2', 'turtle3', 'turtle4']
        self.turtle_publishers = {name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10) for name in self.turtle_names}
        self.turtle_subscribers = {}
        self.turtle_positions = {name: (5.5, 5.5) for name in self.turtle_names}

        # 각 거북이의 pose 구독 생성
        for name in self.turtle_names:
            self.turtle_subscribers[name] = self.create_subscription(
                Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10
            )

        self.timer = self.create_timer(0.5, self.move_turtles)
        self.get_logger().info('TurtleRace Node Started!')

    def pose_callback(self, msg, turtle_name):
        """ 거북이 위치 업데이트 """
        self.turtle_positions[turtle_name] = (msg.x, msg.y, msg.theta)

    def move_turtles(self):
        for name in self.turtle_names:
            msg = Twist()
            x, y, theta = self.turtle_positions[name]

            # 🛑 벽 감지 (turtlesim의 크기는 11x11)
            if x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0:
                self.get_logger().info(f'{name} hit the wall! Trying to escape...')

                # 1️⃣ 후진
                msg.linear.x = -1.0
                msg.angular.z = 0.0
                self.turtle_publishers[name].publish(msg)
                time.sleep(0.5)  # 후진 시간

                # 2️⃣ 방향 전환 (벽을 벗어나도록 강제 회전)
                if x < 1.0:  # 왼쪽 벽
                    turn_angle = math.pi / 2  # 오른쪽으로 90도 회전
                elif x > 10.0:  # 오른쪽 벽
                    turn_angle = -math.pi / 2  # 왼쪽으로 90도 회전
                elif y < 1.0:  # 아래쪽 벽
                    turn_angle = math.pi / 2  # 위쪽으로 회전
                else:  # y > 10.0 (위쪽 벽)
                    turn_angle = -math.pi / 2  # 아래쪽으로 회전

                msg.linear.x = 0.0
                msg.angular.z = turn_angle
                self.turtle_publishers[name].publish(msg)
                time.sleep(1.0)  # 회전 시간

                # 3️⃣ 다시 전진
                msg.linear.x = random.uniform(1.0, 2.0)
                msg.angular.z = 0.0
                self.turtle_publishers[name].publish(msg)

            else:
                # 🔹 벽이 아닐 때는 정상적인 무작위 이동
                msg.linear.x = random.uniform(0.5, 2.0)
                msg.angular.z = random.uniform(-1.0, 1.0)

            self.turtle_publishers[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleRace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
