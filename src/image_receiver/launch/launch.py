#!/usr/bin/env python3
"""
Launch file for image receiver node on WSL host
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate launch description for image receiver
    """

    # Get package directory
    pkg_dir = get_package_share_directory("image_receiver")

    # Declare launch arguments
    enable_display_arg = DeclareLaunchArgument(
        "enable_display", default_value="true", description="Enable local image display"
    )

    image_scale_arg = DeclareLaunchArgument(
        "image_scale",
        default_value="1.0",
        description="Scale factor for received images",
    )

    enable_slam_preprocessing_arg = DeclareLaunchArgument(
        "enable_slam_preprocessing",
        default_value="true",
        description="Enable SLAM preprocessing of images",
    )

    target_fps_arg = DeclareLaunchArgument(
        "target_fps", default_value="15", description="Target FPS for image processing"
    )

    # Parameters file path
    config_file = os.path.join(pkg_dir, "config", "receiver_params.yaml")

    # Image receiver node
    image_receiver_node = Node(
        package="image_receiver",
        executable="image_receiver",
        name="image_receiver",
        output="screen",
        parameters=[
            config_file,
            {
                "enable_display": LaunchConfiguration("enable_display"),
                "image_scale": LaunchConfiguration("image_scale"),
                "enable_slam_preprocessing": LaunchConfiguration(
                    "enable_slam_preprocessing"
                ),
                "target_fps": LaunchConfiguration("target_fps"),
            },
        ],
        remappings=[
            # Remap topics if needed
            # ('/camera/image_raw', '/pi_camera/image_raw'),
        ],
    )

    # RViz2 for visualization (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_dir, "config", "image_viewer.rviz")],
        condition=LaunchConfiguration("enable_display"),
    )

    return LaunchDescription(
        [
            enable_display_arg,
            image_scale_arg,
            enable_slam_preprocessing_arg,
            target_fps_arg,
            image_receiver_node,
            # rviz_node,  # Uncomment to auto-launch RViz2
        ]
    )
