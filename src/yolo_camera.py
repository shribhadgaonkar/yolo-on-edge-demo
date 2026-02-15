#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOCamera(Node):
    def __init__(self):
        super().__init__('yolo_camera')
        self.sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # nano model, fast on Jetson

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image, conf=0.35, verbose=False)
        annotated = results[0].plot()
        cv2.imshow("YOLO Detection - Live Camera", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLOCamera()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()