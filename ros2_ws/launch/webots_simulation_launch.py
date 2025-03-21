from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                './launch/rosbridge_websocket_launch.xml'
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('board_description'), 'launch', 'display.launch.py']),
        ),
        Node(
            package='board_recorder',
            executable='board_recorder',
            name='board_recorder',
            output='screen',
            parameters=[{
                'board_config': './board_configs/TBv2023.json'
            }]
        ),
        Node(
            package='board_joint_pub',
            executable='board_joint_pub',
            name='board_joint_pub',
            namespace='board_joint_pub',
            output='screen',
            parameters=[{
                'task_board_id': 'simulated'
            }]
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('board_webots_sim'), 'launch', 'task_board_launch.py']),
        )
    ])
