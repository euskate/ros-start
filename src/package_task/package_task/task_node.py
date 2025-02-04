import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')
        self.publisher_ = self.create_publisher(String, 'task_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('TaskNode is running')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from TaskNode'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
