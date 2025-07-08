#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YOLOObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detection_node')

        # Load YOLO model
        self.model = YOLO('yolo11n.pt')  # Replace with the correct model path if needed

        # ROS setup
        self.bridge = CvBridge()
        self.rgb_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.annotated_image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run YOLO detection
        results = self.model(frame)
        annotated = frame.copy()

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = results[0].names[cls_id]

                # Draw box and label
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(annotated, f"{label} {conf:.2f}", (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.annotated_image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
