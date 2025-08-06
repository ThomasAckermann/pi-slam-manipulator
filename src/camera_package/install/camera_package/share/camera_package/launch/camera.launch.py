from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Camera publish rate in Hz'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1920',
        description='Image width in pixels'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height', 
        default_value='1080',
        description='Image height in pixels'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='Camera frame ID for TF'
    )
    
    # Camera publisher node
    camera_node = Node(
        package='camera_package',
        executable='camera_publisher',
        name='camera_publisher',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        publish_rate_arg,
        image_width_arg,
        image_height_arg,
        camera_frame_arg,
        camera_node
    ])
