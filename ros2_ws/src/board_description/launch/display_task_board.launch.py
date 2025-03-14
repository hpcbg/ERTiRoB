from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = FindPackageShare('board_description')
    default_model_path = PathJoinSubstitution(['urdf', 'task_board.urdf'])
    default_rviz_config_path = PathJoinSubstitution([urdf_path, 'rviz', 'task_board.rviz'])

    # These parameters are maintained for backwards compatibility
    # gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui')
    # ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'board_description',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            # 'jsp_gui': LaunchConfiguration('gui')
            }.items()
    ))

    ld.add_action(
        Node(
            package='board_joint_pub',
            executable='board_joint_pub',
            name='board_joint_pub',
            namespace='board_joint_pub',
            output='screen',
            parameters=[{
                'task_board_id': '142B2FB1B4E8'
            }]
        ))

    return ld
