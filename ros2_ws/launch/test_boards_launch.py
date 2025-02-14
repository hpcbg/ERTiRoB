from launch import LaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_task_board_simulator',
            executable='test_task_board_simulator',
            name='test_task_board',
            namespace='task_board_test_board_0',
            output='screen',
            parameters=[{
                'task_board_id': 'test_board_0'
            }]
        ),
        Node(
            package='test_task_board_simulator',
            executable='test_task_board_simulator',
            name='test_task_board',
            namespace='task_board_test_board_1',
            output='screen',
            parameters=[{
                'task_board_id': 'test_board_1'
            }]
        ),
    ])
