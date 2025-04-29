import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial #from pyserial
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class JoyPrinter(Node):
    def __init__(self):
        super().__init__('joy_printer')

        # Setup serial port
        try:
            # self.ser = serial.Serial('/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_556393038343517060C1-if00', 9600, timeout=1) # symlink way of pointing. "ls -l /dev/serial/by-id/"
            self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1) #Boron or GPS sometimes?
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # Characters to cycle through for testing
        # self.commands = ['f', 'b', 'l', 'r']

        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)
        self.get_logger().info("Heartbeat Started")

        # Timer callback every 2 seconds for commands
        self.timer = self.create_timer(0.2, self.send_command)

        # subscriber setup to joystick topic
        self.latest_joy_msg = None
        self.subscription = self.create_subscription(
            Joy,
            'joy',               # topic name
            self.joy_callback,   # callback function
            10                   # queue size
        )
        self.get_logger().info("Listening to /joy...")

        self.heading_error_sub = self.create_subscription(
            Float32, 
            'guardian/heading_error',
            self.heading_error_callback,   # callback function
            10                   # queue size
        )
        self.distance_sub = self.create_subscription(
            Float32, 
            'guardian/distance',
            self.distance_callback,   # callback function
            10                   # queue size
        )

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

        # testing fake waypoint
        self.target_waypoint = Point()
        self.target_waypoint.x = 42.034752
        self.target_waypoint.y = -87.912656
        self.target_waypoint.z = 0.0 #unused
        self.heading_error = -999
        self.distance = -999
        self.target_waypoint_pub = self.create_publisher(Point, '/guardian/target_gps', 10)

        # flag for autonomous mode
        self.autonomous_mode = False


    def send_heartbeat(self):
        if self.ser and self.ser.is_open:
            try:
                # self.ser.write(b'h\n')
                self.get_logger().debug("Sent heartbeat: H")
            except serial.SerialException as e:
                self.get_logger().error(f"Heartbeat failed: {e}")
                self.ser.close()
                self.ser = None

    def send_command(self):

        if self.latest_joy_msg is None:
            self.get_logger().warn("No joystick input received yet.")
            return  # Skip sending command until we have data
            
        if self.autonomous_mode == True:
            DISTANCE_TRESH = 10 # 10 meters to the target
            DEG_TRESH = 10

            if (self.distance != -999) & (self.heading_error != -999):
                # start autonomous if curr distance is further from target
                if self.distance > DISTANCE_TRESH:
                    if self.heading_error > DEG_TRESH: # Robot is CCW, need to rotate CW
                        command = 'r'
                    elif self.heading_error < -DEG_TRESH: # Robot is CW, need to rotate CCW
                        command = 'l'
                    else:
                        command = 'f'
                else:
                    command = 's'

        else:
            left_stick_x = self.latest_joy_msg.axes[6] # left, right
            left_stick_y = self.latest_joy_msg.axes[7] # forward, backward

            if left_stick_y == 1:
                command = 'f'
            elif left_stick_y == -1:
                command = 'b'
            elif left_stick_x == 1:
                command = 'l'
            elif left_stick_x == -1:
                command = 'r'
            else:
                command = 's'

        # ----- Digging and Water Pump ------
        LB_button = self.latest_joy_msg.buttons[4] # LB, for digging
        RB_button = self.latest_joy_msg.buttons[5] # RB, for water
        if LB_button == 1:
            command = 'd'
        if RB_button == 1:
            command = 'w'

        if self.ser and self.ser.is_open:
            self.ser.write((command + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent command: {command}, heading error {self.heading_error:0.2f}")
        else:
            self.get_logger().warn("Serial not open.")


        # pass

    def joy_callback(self, msg):        
        # Print message from joy topic
        # self.get_logger().info(f"Received Joy msg:\n{msg}")
        self.latest_joy_msg = msg
        left_stick_x = msg.axes[6] # left, right
        left_stick_y = msg.axes[7] # forward, backward
        LB_button = msg.buttons[4]
        RB_button = msg.buttons[5]

        self.target_waypoint_pub.publish(self.target_waypoint)

        button_a = msg.buttons[0] # hold button A to trigger autonomous mode
        if button_a == 1:
            self.autonomous_mode = True 
        # self.get_logger().info(f"Published target GPS: lat={self.target_waypoint.x}, lon={self.target_waypoint.y}")
        # self.get_logger().info(
        #     f"LX: {left_stick_x:.2f}, LY: {left_stick_y:.2f}"
        # )

        # pass

    def heading_error_callback(self, msg):
        self.heading_error = msg.data
        # self.get_logger().info(f"heading error: {self.heading_error}")

    def distance_callback(self, msg):
        self.distance = msg.data
        # self.get_logger().info(f"distance : {self.distance}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()