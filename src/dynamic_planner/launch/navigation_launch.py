from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'dynamic_planner'
    pkg_share = get_package_share_directory(package_name)

    # Paths to files
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_bot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'map.world')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'config', 'empty_map.yaml')

    # ✅ Start Gazebo with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file  # ✅ Ensure Gazebo loads your world
        }.items(),
    )

    # ✅ Spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
        output='screen'
    )

    # ✅ Start Nav2 with required parameters
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'map': map_file,
            'use_sim_time': 'true'  # ✅ Ensure Nav2 uses simulated time
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        nav2_bringup,  # ✅ Start Nav2 automatically
    ])
