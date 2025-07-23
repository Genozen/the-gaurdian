import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy

import torch
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
import os

class FireDetectorNode(Node):
    def __init__(self):
        super().__init__('fire_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        model_path = os.path.join(
        get_package_share_directory('guardian_control'),
        'models',
        'fire_epoch_50_batch_6.engine' #'fire_epoch5_batch_4_imgsize_640.engine' # 'yolo11n.engine'
        )

        self.model = YOLO(model_path)

        self.last_log = self.get_clock().now()   # tiny FPS logger
        self.get_logger().info("Fire Detector Node Started")

        self.isFire = Bool()
        self.isFire.data = False
        self.isFire_pub = self.create_publisher(Bool, '/guardian/isFire', 10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            flipped = cv2.flip(frame, 0)   # Flip vertically (upside down)
            # TODO: Add fire detection logic here
            results = self.model(flipped, verbose=False) # verbose=false silence the messages
            # self.get_logger().info(f"Detections: {results}")

            for r in results:
                if r.boxes is not None:
                    for box in r.boxes:
                        # Get box coordinates as integers
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)  # [x1, y1, x2, y2]
                        conf = box.conf[0].item()
                        
                        if conf >= 0.50:
                            cls_id = int(box.cls[0].item())
                            label = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)

                            ## Draw rectangle
                            cv2.rectangle(flipped, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                            ## Draw label
                            cv2.putText(flipped, f"{label} {conf:.2f}", (xyxy[0], xyxy[1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            self.isFire.data = True
                            self.isFire_pub.publish(self.isFire)
                            self.get_logger().info(f"Confidence: {conf:.2f}")
                        else:
                            self.isFire.data = False
                            

            cv2.imshow("Fire Detection", flipped)
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
