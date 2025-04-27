import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu


class GuardianNavNode(Node):
    def __init__(self):
        super().__init__('guardian_nav_node')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/shield/gps',
            self.gps_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/shield/imu',
            self.imu_callback,
            10
        )


    def gps_callback(self, msg):
        self.get_logger().info(f"GPS: lat={msg.latitutude:.6f}, lon={msg.longitude:.6f}")

    def imu_callback(self, msg):
        self.get_logger().info(f"IMU: orientation_z = {msg.orientation.z:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = GuardianNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()