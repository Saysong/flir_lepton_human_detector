import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ThresholdHumanDetectorNode(Node):
    def __init__(self):
        super().__init__('threshold_human_detector')
        self.subscription = self.create_subscription(
            Image,
            '/thermal_image',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/person_warning', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Threshold-based human detector initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            if self.detect_human_from_rgb(frame):
                self.publisher.publish(String(data="⚠️ Person detected (contour threshold)"))
                self.get_logger().warn("⚠️ Person detected!")
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {str(e)}")

    def detect_human_from_rgb(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 8 < area < 350:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdHumanDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
