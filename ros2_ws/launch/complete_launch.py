from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='board_recorder',
            executable='board_recorder',
            name='board_recorder',
            output='screen',
            parameters=[{
                'board_config': './board_configs/default.json'
            }]
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        )
    ])
