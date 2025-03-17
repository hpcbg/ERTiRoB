from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = FindPackageShare('board_description')

    default_model_path = PathJoinSubstitution(['urdf', 'task_board.urdf'])

    default_rviz_config_path = PathJoinSubstitution(
        [urdf_path, 'rviz', 'task_board.rviz'])
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    urdf_launch_package = FindPackageShare('urdf_launch')

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution(
            [urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'board_description',
            'urdf_package_path': LaunchConfiguration('model')}.items()
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))

    return ld
