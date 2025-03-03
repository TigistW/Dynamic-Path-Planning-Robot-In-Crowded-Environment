#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
from ament_index_python.packages import get_package_share_directory


class SpawnRobot(Node):
    def __init__(self):
        super().__init__('spawn_robot_node')
        package_share = get_package_share_directory('dynamic_planner')
        self.world_file = os.path.join(package_share, 'worlds', 'map.world')
        self.urdf_file = os.path.join(package_share, 'urdf', 'my_bot.urdf')

        if not os.path.exists(self.world_file):
            self.get_logger().error(f"World file not found: {self.world_file}")
            return

        if not os.path.exists(self.urdf_file):
            self.get_logger().error(f"URDF file not found: {self.urdf_file}")
            return

        self.get_logger().info(f"Using world file: {self.world_file}")
        self.get_logger().info(f"Using URDF file: {self.urdf_file}")

        
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

        
        self.get_logger().info("Waiting for the /spawn_entity service...")
        self.client.wait_for_service()

       
        self.spawn_robot()

    def spawn_robot(self):
        # Read URDF
        with open(self.urdf_file, 'r') as urdf_file:
            robot_description = urdf_file.read()

        # Create the request
        request = SpawnEntity.Request()
        request.xml = robot_description
        request.name = 'simple_robot'
        request.initial_pose.position.x = 2.0
        request.initial_pose.position.y = 3.0
        request.initial_pose.position.z = 0.1
        request.initial_pose.orientation.w = 1.0

        # Call the service to spawn the robot
        self.get_logger().info("Spawning the robot...")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned robot: {request.name}")
        else:
            self.get_logger().error("Failed to spawn robot")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
