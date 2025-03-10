from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='recording_player',
            executable='recording_player',
            name='recording_player',
            output='screen',
            parameters=[{
                'task_board_id': 'recording',
                'board_config': './board_configs/TBv2023.json',
                'recording': './recordings/robothon_2023.json',
                'timeout': 5
            }]
        )
    ])
