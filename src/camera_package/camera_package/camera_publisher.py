#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import subprocess
import tempfile
import os
from pathlib import Path
import yaml

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('image_width', 512)
        self.declare_parameter('image_height', 512)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, camera_topic, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Temporary directory for images
        self.temp_dir = tempfile.mkdtemp()
        self.temp_image_path = os.path.join(self.temp_dir, 'capture.jpg')
        
        # Timer for image capture and publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.capture_and_publish)
        
        # Camera info (you'll need to calibrate your camera to get accurate values)
        self.camera_info = self.create_camera_info()
        
        self.get_logger().info(f'Camera publisher started at {self.publish_rate} Hz')
        self.get_logger().info(f'Publishing to: {camera_topic}')
        self.get_logger().info(f'Image resolution: {self.image_width}x{self.image_height}')

    def capture_image(self) -> bool:
        """Capture image using rpicam-still"""
        try:
            cmd = [
                "rpicam-still",
                "--immediate",
                "--width", str(self.image_width),
                "--height", str(self.image_height),
                "--encoding", "jpg",
                "--quality", "90",
                "-o", self.temp_image_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                return True
            else:
                self.get_logger().error(f'rpicam-still failed: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Camera capture timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Camera capture error: {str(e)}')
            return False

    def capture_and_publish(self):
        """Main callback to capture and publish image"""
        if not self.capture_image():
            return
        
        try:
            # Read the captured image
            cv_image = cv2.imread(self.temp_image_path)
            if cv_image is None:
                self.get_logger().error('Failed to load captured image')
                return
            
            # Convert BGR to RGB (OpenCV uses BGR by default)
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image_rgb, encoding='rgb8')
            
            # Set header information
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = self.frame_id
            
            # Publish image
            self.image_publisher.publish(ros_image)
            
            # Update and publish camera info
            self.camera_info.header = ros_image.header
            self.camera_info_publisher.publish(self.camera_info)
            
            # Clean up temporary file
            if os.path.exists(self.temp_image_path):
                os.remove(self.temp_image_path)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

    def create_camera_info(self):
        """Create camera info message with basic parameters"""
        camera_info = CameraInfo()
        
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # These are placeholder values - you should calibrate your camera
        # for accurate intrinsic parameters
        fx = fy = 800.0  # Focal length (approximate)
        cx = self.image_width / 2.0   # Principal point x
        cy = self.image_height / 2.0  # Principal point y
        
        # Camera intrinsic matrix
        camera_info.k = [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix (same as K for monocular camera)
        camera_info.p = [
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion parameters (assuming no distortion - you should calibrate)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = "plumb_bob"
        
        # Rectification matrix (identity for single camera)
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        return camera_info

    def destroy_node(self):
        """Clean up when node is destroyed"""
        # Clean up temporary directory
        try:
            if os.path.exists(self.temp_dir):
                import shutil
                shutil.rmtree(self.temp_dir)
        except Exception as e:
            self.get_logger().warn(f'Error cleaning up temp directory: {str(e)}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
