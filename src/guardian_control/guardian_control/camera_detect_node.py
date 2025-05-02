import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import torch
from ultralytics import YOLO
from PIL import Image

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Fire Detector Node Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # TODO: Add fire detection logic here
            flipped = cv2.flip(frame, 0)   # Flip vertically (upside down)
            cv2.imshow("Camera View", flipped)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
