import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

from tf_transformations import euler_from_quaternion
import math

# normalize angle
def normalize_angle_180(deg):
    angle = (deg + 180) % 360 - 180
    return angle

def normalize_angle_360(deg):
    return (deg + 360) % 360

# This assume earth is flat for two GPS points < 100 meters
# https://stackoverflow.com/a/64747209
def calculate_bearing(curr_lat, curr_lon, target_lat, target_lon):
    curr_lat = math.radians(curr_lat)
    curr_lon = math.radians(curr_lon)
    target_lat = math.radians(target_lat)
    target_lon = math.radians(target_lon)

    delta_lon = target_lon - curr_lon
    x = math.cos(target_lat) * math.sin(delta_lon)
    y = math.cos(curr_lat) * math.sin(target_lat) - math.sin(curr_lat) * math.cos(target_lat) * math.cos(delta_lon)

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg_norm = normalize_angle_180(bearing_deg)

    return bearing_deg_norm

# Computes the distance of two GPS pionts
# https://stackoverflow.com/questions/27928/calculate-distance-between-two-latitude-longitude-points-haversine-formula
def calculate_distance(curr_lat, curr_lon, target_lat, target_lon):
    """
    Calculate great-circle distance between two points using haversine formula.
    Output in meters.
    """
    R = 6371000  # Earth's radius in meters

    curr_lat_rad = math.radians(curr_lat)
    curr_lon_rad = math.radians(curr_lon)
    target_lat_rad = math.radians(target_lat)
    lon2_rad = math.radians(target_lon)

    delta_lat = target_lat_rad - curr_lat_rad
    delta_lon = lon2_rad - curr_lon_rad

    a = math.sin(delta_lat / 2) ** 2 + \
        math.cos(curr_lat_rad) * math.cos(target_lat_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    return distance
    

def compute_heading_error(target_deg, curr_deg):
    deg_error = curr_deg - target_deg
    return deg_error

class GuardianNavNode(Node):
    def __init__(self):
        super().__init__('guardian_nav_node')

        self.target_lat = -1
        self.target_lon = -1
        self.curr_lat = -1
        self.curr_lon = -1
        self.yaw_deg = float(-999) # degree
        self.heading_error = float(-999) # degree
        self.distance = float(-999) # meters

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

        self.target_gps_sub = self.create_subscription(
            Point, 
            '/guardian/target_gps',
            self.target_gps_callback,
            10
        )

        self.yaw_pub = self.create_publisher(Float32, 'guardian/yaw_deg', 10)
        self.heading_error_pub = self.create_publisher(Float32, 'guardian/heading_error', 10)
        self.distance_pub = self.create_publisher(Float32, 'guardian/distance', 10)

    def gps_callback(self, msg):
        # lat1 = 42.034495
        # lon1 = -87.912657
        # lat2 = 42.034752
        # lon2 = -87.912656
        # lat3 = 42.034752
        # lon3 = -87.912473
        # lat4 = 42.034494
        # lon4 = -87.912473
        # lat5 = 42.034751
        # lon5 = -87.912800
        # lat6 = 42.034496
        # lon6 = -87.912789

        self.curr_lat = 42.034495
        self.curr_lon = -87.912657

        # ----------- Get actual GNSS data
        # self.curr_lat = msg.latitude
        # self.curr_lon = msg.longitude

        # self.get_logger().info(f"GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")


        # bearing_deg1 = calculate_bearing(lat1, lon1, lat2, lon2)
        # bearing_deg2 = calculate_bearing(lat1, lon1, lat5, lon5)
        # bearing_deg3 = calculate_bearing(lat1, lon1, lat6, lon6)
        # self.get_logger().info(f"Bearing (degree): {bearing_deg1:.2f} {bearing_deg2:.2f} {bearing_deg3:.2f}")

        # heading_error1 = compute_heading_error(0, bearing_deg1)
        # heading_error2 = compute_heading_error(0, bearing_deg2)
        # heading_error3 = compute_heading_error(0, bearing_deg3)
        # self.get_logger().info(f"Heading err (degree): {heading_error1:.2f} {heading_error2:.2f} {heading_error3:.2f}")

        # print(bearing_deg1, bearing_deg2, bearing_deg3)


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
        self.yaw_deg = yaw_degrees

        # heading_error = compute_heading_error(0, yaw_degrees)

        # self.yaw_pub.publish(Float32(data=yaw_degrees_norm))
        # self.get_logger().info(f"Heading error (degree): {0}, {yaw_degrees}, {heading_error:.2f}")
        if (self.target_lat != -1) & (self.target_lon != -1) & (self.curr_lat != -1) & (self.curr_lon != -1):
            target_bearing = calculate_bearing(self.curr_lat, self.curr_lon, self.target_lat, self.target_lon)
            self.distance = calculate_distance(self.curr_lat, self.curr_lon, self.target_lat, self.target_lon)
            if self.yaw_deg != -999:
                self.heading_error = compute_heading_error(target_bearing, self.yaw_deg)
        
        self.heading_error_pub.publish(Float32(data=self.heading_error))
        self.distance_pub.publish(Float32(data=self.distance))

    def target_gps_callback(self, msg):
        self.target_lat = msg.x
        self.target_lon = msg.y

        self.get_logger().info(f"GPS computing: {self.target_lat} | {self.target_lon} | {self.curr_lat} | {self.curr_lon}")
        # self.get_logger().info(f"Yaw {self.yaw_deg} | bearing {target_bearing} | heading err {self.heading_error}")

def main(args=None):
    rclpy.init(args=args)
    node = GuardianNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()