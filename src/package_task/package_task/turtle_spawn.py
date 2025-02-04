#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import time

class TurtleSpawner(Node):
    def __init__(self):
        # 노드 이름 'turtle_spawner'로 초기화
        super().__init__('turtle_spawner')
        # 생성할 거북이들의 이름 (기본 거북이(turtle1)는 이미 있음)
        self.turtle_names = ['turtle2', 'turtle3', 'turtle4']
        # 각 거북이의 초기 위치 (x, y)
        self.positions = [(2.0, 2.0), (5.0, 5.0), (8.0, 2.0)]
        # Spawn 서비스를 위한 클라이언트 생성
        self.client = self.create_client(Spawn, '/spawn')

        # Spawn 서비스가 사용 가능할 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        
        # 모든 거북이를 생성하는 함수 호출
        self.spawn_turtles()

    def spawn_turtles(self):
        """
        설정된 이름과 위치로 거북이를 생성합니다.
        각 요청은 Spawn 서비스를 통해 비동기로 호출되며, 결과를 기다립니다.
        """
        for index, name in enumerate(self.turtle_names):
            request = Spawn.Request()
            request.x = self.positions[index][0]
            request.y = self.positions[index][1]
            request.theta = 0.0  # 초기 방향은 0으로 설정
            request.name = name

            # 비동기 방식으로 서비스 호출
            future = self.client.call_async(request)
            # 서비스 응답이 올 때까지 대기
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Spawned turtle: {future.result().name}")
            else:
                self.get_logger().error("Failed to spawn turtle.")

def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    # 거북이가 생성될 시간을 고려하여 잠시 대기
    time.sleep(1)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
