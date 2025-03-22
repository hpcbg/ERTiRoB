import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('board_webots_sim')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'task_board.urdf')

    webots_supervisor = Ros2SupervisorLauncher(
        port='1234'
    )

    task_board_driver = WebotsController(
        robot_name='TaskBoard',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    return LaunchDescription([
        # webots,
        webots_supervisor,
        task_board_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_supervisor,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
