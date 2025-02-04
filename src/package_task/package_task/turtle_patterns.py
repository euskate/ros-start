#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

class TurtlePatterns(Node):
    def __init__(self):
        # 노드 이름 'turtle_patterns'로 초기화
        super().__init__('turtle_patterns')
        # 패턴을 그릴 거북이 목록 (세 마리)
        self.turtle_names = ['turtle1', 'turtle2', 'turtle3']
        
        # 각 거북이에 대해 cmd_vel 퍼블리셔 생성
        self.turtle_publishers = {}
        for name in self.turtle_names:
            self.turtle_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        
        # 각 거북이의 현재 위치와 방향 정보를 구독하기 위한 subscriber 생성
        self.turtle_subscribers = {}
        for name in self.turtle_names:
            self.turtle_subscribers[name] = self.create_subscription(
                Pose, f'/{name}/pose', lambda msg, n=name: self.pose_callback(msg, n), 10
            )
        
        # 각 거북이의 초기 위치 및 방향 (x, y, theta)
        self.turtle_positions = {name: (5.5, 5.5, 0.0) for name in self.turtle_names}
        
        self.get_logger().info('TurtlePatterns node started.')

    def pose_callback(self, msg, turtle_name):
        """
        구독한 Pose 메시지를 통해 거북이의 현재 위치와 방향을 업데이트합니다.
        :param msg: Pose 메시지 (x, y, theta)
        :param turtle_name: 해당 거북이의 이름
        """
        self.turtle_positions[turtle_name] = (msg.x, msg.y, msg.theta)

    def move_turtle(self, name, linear, angular, duration):
        """
        지정된 시간(duration) 동안 거북이에게 주어진 선속도(linear)와 각속도(angular)로 이동하도록 명령합니다.
        만약 이동 중 벽 근처에 도달하면 장애물 회피 동작을 수행합니다.
        :param name: 거북이 이름
        :param linear: 선속도 명령 값
        :param angular: 각속도 명령 값
        :param duration: 동작 지속 시간 (초 단위)
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        end_time = time.time() + duration

        while time.time() < end_time:
            # 현재 위치 및 방향 확인
            x, y, theta = self.turtle_positions[name]

            # 벽과의 충돌 감지 (창의 범위: 약 1.0 ~ 10.0)
            if x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0:
                self.get_logger().info(f'{name} hit the wall during pattern drawing! Initiating escape maneuver.')

                # [Step 1] 후진: 벽에서 멀어지기 위해 잠시 후진
                twist_msg.linear.x = -1.0
                twist_msg.angular.z = 0.0
                self.turtle_publishers[name].publish(twist_msg)
                time.sleep(0.5)

                # [Step 2] 90도 회전: 벽에서 벗어날 수 있는 방향으로 회전
                twist_msg.linear.x = 0.0
                if x < 1.0:          # 왼쪽 벽: 오른쪽으로 90도 회전
                    twist_msg.angular.z = math.pi / 2
                elif x > 10.0:       # 오른쪽 벽: 왼쪽으로 90도 회전
                    twist_msg.angular.z = -math.pi / 2
                elif y < 1.0:        # 아래쪽 벽: 위쪽으로 90도 회전
                    twist_msg.angular.z = math.pi / 2
                else:                # 위쪽 벽: 아래쪽으로 90도 회전
                    twist_msg.angular.z = -math.pi / 2
                self.turtle_publishers[name].publish(twist_msg)
                time.sleep(1.0)

                # [Step 3] 전진: 회전 후, 앞으로 전진
                twist_msg.linear.x = 1.0
                twist_msg.angular.z = 0.0
                self.turtle_publishers[name].publish(twist_msg)
            else:
                # 벽이 없으면 의도한 속도로 이동 명령을 계속 발행
                self.turtle_publishers[name].publish(twist_msg)
            time.sleep(0.1)

    def draw_circle(self, name):
        """
        거북이가 원을 그리도록 명령합니다.
        원은 일정 선속도와 각속도로 동시에 이동하여 그립니다.
        :param name: 거북이 이름
        """
        self.move_turtle(name, linear=1.0, angular=1.0, duration=6.0)

    def draw_square(self, name):
        """
        거북이가 사각형을 그리도록 명령합니다.
        한 변을 그리고, 90도 회전하는 동작을 4회 반복합니다.
        :param name: 거북이 이름
        """
        for _ in range(4):
            self.move_turtle(name, linear=1.0, angular=0.0, duration=2.0)  # 한 변 이동
            self.move_turtle(name, linear=0.0, angular=math.pi/2, duration=1.0)  # 90도 회전

    def draw_triangle(self, name):
        """
        거북이가 삼각형을 그리도록 명령합니다.
        한 변을 그리고, 120도 회전하는 동작을 3회 반복합니다.
        :param name: 거북이 이름
        """
        for _ in range(3):
            self.move_turtle(name, linear=1.0, angular=0.0, duration=2.0)  # 한 변 이동
            self.move_turtle(name, linear=0.0, angular=2*math.pi/3, duration=1.0)  # 120도 회전

    def start_drawing(self):
        """
        각 거북이에게 다음의 패턴을 차례대로 그리도록 명령합니다.
          - turtle1: 원
          - turtle2: 사각형
          - turtle3: 삼각형
        """
        self.draw_circle('turtle1')
        self.draw_square('turtle2')
        self.draw_triangle('turtle3')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePatterns()
    node.start_drawing()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
