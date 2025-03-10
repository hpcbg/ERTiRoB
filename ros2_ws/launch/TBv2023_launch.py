from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                './launch/rosbridge_websocket_launch.xml'
            )
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
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        ),
    ])
