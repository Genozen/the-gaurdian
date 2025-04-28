import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32

from tf_transformations import euler_from_quaternion
import math

# This assume earth is flat for two GPS points < 100 meters
# https://stackoverflow.com/a/64747209
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    delta_lon = lon2 - lon1
    x = math.cos(lat2) * math.sin(delta_lon)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad) + 360 % 360 # normalize 0 - 360

    return bearing_deg


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
        lat1 = 42.034495
        lon1 = -87.912657
        lat2 = 42.034752
        lon2 = -87.912656
        lat3 = 42.034752
        lon3 = -87.912473
        lat4 = 42.034494
        lon4 = -87.912473

        bearing_deg1 = calculate_bearing(lat1, lon1, lat2, lon2)
        bearing_deg2 = calculate_bearing(lat1, lon1, lat3, lon3)
        bearing_deg3 = calculate_bearing(lat1, lon1, lat4, lon4)

        # print(bearing_deg1, bearing_deg2, bearing_deg3)
        self.get_logger().info(f"Bearing (degree): {bearing_deg1:.2f} {bearing_deg2:.2f} {bearing_deg3:.2f}")


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
        # self.get_logger().info(f"Yaw (degree): {yaw_degrees:.2f}")
        self.yaw_pub.publish(Float32(data=yaw_degrees))


def main(args=None):
    rclpy.init(args=args)
    node = GuardianNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()