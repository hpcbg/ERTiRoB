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
                'task_board_id': 'test_board_0',
                'board_config': './board_configs/default.json',
                'recording': './recordings/test_recording.json',
                'timeout': 5
            }]
        )
    ])
