from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'dynamic_planner'

    # Paths
    nav2_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2_params.yaml'
    )
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'my_bot.urdf'
    )

    # Robot spawning
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot', '-file', urdf_file],
        output='screen'
    )

    nav2_bringup = Node(
        package='nav2_bringup',
        executable='navigation_launch.py',
        output='screen',
        parameters=[nav2_params_file],
    )

    return LaunchDescription([
        spawn_robot,
        nav2_bringup,
    ])