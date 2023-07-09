from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2-gst-test',
            executable='server_node',
            output='screen',
            emulate_tty=True),
    ])