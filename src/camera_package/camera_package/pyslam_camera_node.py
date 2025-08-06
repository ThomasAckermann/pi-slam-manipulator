#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
import pyslam



class PySLAMCameraNode(Node):
    def __init__(self):
        super().__init__('pyslam_camera_node')
        
        if not pyslam_available:
            self.get_logger().error("PySLAM not available! Check installation.")
            return
        
        # ROS setup
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/slam/pose', 10)
        
        self.bridge = CvBridge()
        
        # PySLAM setup
        self.setup_pyslam()
        
        # State variables
        self.frame_id = 0
        self.is_initialized = False
        
        self.get_logger().info('PySLAM Camera Node started')

    def setup_pyslam(self):
        """Initialize PySLAM components"""
        try:
            # Camera parameters (you should calibrate these!)
            # These are approximate for Pi Camera v2
            width = 1920
            height = 1080
            fx = 800.0  # focal length x
            fy = 800.0  # focal length y
            cx = width / 2.0   # principal point x
            cy = height / 2.0  # principal point y
            
            # Create camera object
            self.camera = PinholeCamera(width, height, fx, fy, cx, cy)
            
            # Initialize SLAM system
            # You can choose different configurations:
            # 'ORB', 'SIFT', 'SURF', 'FAST', 'BRISK', etc.
            feature_type = 'ORB'
            
            # Create SLAM system
            self.slam = Slam(self.camera, feature_type)
            
            self.get_logger().info(f'PySLAM initialized with {feature_type} features')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PySLAM: {str(e)}')
            self.slam = None

    def image_callback(self, msg):
        """Process incoming ROS images with PySLAM"""
        if not pyslam_available or self.slam is None:
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # Convert to grayscale (PySLAM typically works with grayscale)
            if len(cv_image.shape) == 3:
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            else:
                gray_image = cv_image
                
            # Get timestamp
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Process frame with PySLAM
            self.process_frame(gray_image, timestamp)
            
            self.frame_id += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

    def process_frame(self, image, timestamp):
        """Process a single frame with PySLAM"""
        try:
            # Track features and estimate pose
            self.slam.track(image, self.frame_id, timestamp)
            
            # Get current pose if available
            if hasattr(self.slam, 'get_pose') and self.slam.tracking_is_ok:
                pose_matrix = self.slam.get_pose()
                if pose_matrix is not None:
                    self.publish_pose(pose_matrix, timestamp)
            
            # Log status periodically
            if self.frame_id % 30 == 0:  # Every 30 frames
                status = "TRACKING" if self.slam.tracking_is_ok else "LOST"
                num_features = len(self.slam.curr_keypoints) if hasattr(self.slam, 'curr_keypoints') else 0
                self.get_logger().info(f'Frame {self.frame_id}: {status}, Features: {num_features}')
                
        except Exception as e:
            self.get_logger().warn(f'SLAM processing failed: {str(e)}')

    def publish_pose(self, pose_matrix, timestamp):
        """Publish estimated pose as ROS message"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'odom'
            
            # Extract position from 4x4 transformation matrix
            if pose_matrix.shape == (4, 4):
                pose_msg.pose.position.x = float(pose_matrix[0, 3])
                pose_msg.pose.position.y = float(pose_matrix[1, 3]) 
                pose_msg.pose.position.z = float(pose_matrix[2, 3])
                
                # Extract rotation matrix and convert to quaternion
                rotation_matrix = pose_matrix[:3, :3]
                # Simple conversion - you might want to use scipy.spatial.transform
                # For now, just set identity quaternion
                pose_msg.pose.orientation.w = 1.0
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                
                self.pose_publisher.publish(pose_msg)
                
        except Exception as e:
            self.get_logger().warn(f'Failed to publish pose: {str(e)}')

    def shutdown(self):
        """Clean shutdown"""
        if hasattr(self, 'slam') and self.slam:
            try:
                # Save map or perform cleanup if needed
                self.get_logger().info('Shutting down PySLAM...')
            except Exception as e:
                self.get_logger().warn(f'Shutdown error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        pyslam_node = PySLAMCameraNode()
        rclpy.spin(pyslam_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'pyslam_node' in locals():
            pyslam_node.shutdown()
            pyslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
