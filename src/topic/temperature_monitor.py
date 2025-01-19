import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.data > 25.0:
            self.get_logger().warning(f'High temperature detected: {msg.data:.2f}°C')
        else:
            self.get_logger().info(f'Temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
