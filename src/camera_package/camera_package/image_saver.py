# Save as image_saver.py on your Pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.save_counter = 0
        
        # Create directory
        os.makedirs('/tmp/camera_images', exist_ok=True)
        
    def image_callback(self, msg):
        if self.save_counter % 30 == 0:  # Save every 30th frame (roughly every 3 seconds at 10Hz)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            filename = f'/tmp/camera_images/image_{self.save_counter:06d}.jpg'
            cv2.imwrite(filename, cv_image_bgr)
            self.get_logger().info(f'Saved {filename}')
            
        self.save_counter += 1

def main():
    rclpy.init()
    saver = ImageSaver()
    rclpy.spin(saver)

if __name__ == '__main__':
    main()
