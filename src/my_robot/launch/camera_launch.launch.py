from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_p',
            name='camera_p_node',
            output='screen'
        ),
        Node(
            package='camera',
            executable='camera_s',
            name='camera_s_node',
            output='screen'
        )
    ])
