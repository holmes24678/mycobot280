#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # <-- Replace this with your topic
            self.image_callback,
            10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)  # Needed to keep imshow window responsive
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
