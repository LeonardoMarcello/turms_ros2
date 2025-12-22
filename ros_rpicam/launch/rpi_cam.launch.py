from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('ros_rpicam'), 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='ros_rpicam',
            executable='rpi_cam_node',
            name='rpi_cam',
            parameters=[config],
            output='screen'
        )
    ])
