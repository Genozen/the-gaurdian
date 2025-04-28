import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32

from tf_transformations import euler_from_quaternion
import math

class GuardianNavNode(Node):
    def __init__(self):
        super().__init__('guardian_nav_node')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/shield/gps',
            self.gps_callback,
            qos_profile #10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/shield/imu',
            self.imu_callback,
            qos_profile #10
        )

        self.yaw_pub = self.create_publisher(Float32, 'guardian/yaw_deg', 10)


    def gps_callback(self, msg):
        # self.get_logger().info(f"GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
        pass

    def imu_callback(self, msg):
        # self.get_logger().info(f"IMU: orientation_z = {msg.orientation.z:.6f}")
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        yaw_degrees = math.degrees(yaw)
        self.get_logger().info(f"Yaw (degree): {yaw_degrees:.2f}")
        self.yaw_pub.publish(Float32(data=yaw_degrees))
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GuardianNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()