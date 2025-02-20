import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'dynamic_planner'  # Your package name

    # Paths to files
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_bot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'map.world')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )

    # Spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # Start the Custom DWA Controller
    dwa_controller = Node(
        package=package_name,
        executable='dwa_controller',
        output='screen'
    )

    # Start the Camera Subscriber Node
    camera_subscriber = Node(
        package=package_name,
        executable='camera_subscriber',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        dwa_controller,
        camera_subscriber,  # Added Camera Subscriber Node
    ])
