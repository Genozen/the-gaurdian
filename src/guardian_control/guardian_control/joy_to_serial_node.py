import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPrinter(Node):
    def __init__(self):
        super().__init__('joy_printer')
        self.subscription = self.create_subscription(
            Joy,
            'joy',               # topic name
            self.joy_callback,   # callback function
            10                   # queue size
        )
        self.get_logger().info("Listening to /joy...")

    def joy_callback(self, msg):
        self.get_logger().info(f"Received Joy msg:\n{msg}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()