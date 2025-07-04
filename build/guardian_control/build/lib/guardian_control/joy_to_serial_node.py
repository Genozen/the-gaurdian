import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial #from pyserial

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
        self.commands = ['f', 'b', 'l', 'r']
        self.current_index = 0

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
            
        left_stick_x = self.latest_joy_msg.axes[6] # left, right
        left_stick_y = self.latest_joy_msg.axes[7] # forward, backward
        LB_button = self.latest_joy_msg.buttons[4] # LB, for digging
        RB_button = self.latest_joy_msg.buttons[5] # RB, for water

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
        print(command)

        # Turn digging motor on
        if LB_button == 1:
            command = 'd'

        if RB_button == 1:
            command = 'w'

        if self.ser and self.ser.is_open:
            # command = self.commands[self.current_index]
            self.ser.write((command + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent command: {command}")
            self.current_index = (self.current_index + 1) % len(self.commands)
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

        # self.get_logger().info(
        #     f"LX: {left_stick_x:.2f}, LY: {left_stick_y:.2f}"
        # )

        # pass

def main(args=None):
    rclpy.init(args=args)
    node = JoyPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()