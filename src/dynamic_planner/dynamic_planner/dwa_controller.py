import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

class RobotConfig:
    def __init__(self):
        # Velocity limits
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.acceleration_limit = 0.2  # m/s²
        self.angular_acceleration_limit = 0.5  # rad/s²
        self.dt = 0.1  # Time step for trajectory prediction
        self.predict_time = 2.0  # Time horizon for trajectory evaluation
        self.obstacle_radius = 0.3  # Distance around obstacle considered unsafe


import math

class DWAPlanner:
    def __init__(self, config):
        self.config = config

    def compute_trajectory_cost(self, velocities, goal, robot_pose, obstacles):
        """
        Compute cost for a trajectory given a velocity sample.
        """
        x, y, theta = robot_pose
        v, w = velocities

        trajectory_cost = 0.0
        for t in np.arange(0, self.config.predict_time, self.config.dt):
            # Simulate robot position
            x += v * math.cos(theta) * self.config.dt
            y += v * math.sin(theta) * self.config.dt
            theta += w * self.config.dt

            # Goal distance cost
            distance_to_goal = math.hypot(goal[0] - x, goal[1] - y)

            # Obstacle distance cost
            min_obstacle_dist = min(
                [math.hypot(x - obs[0], y - obs[1]) for obs in obstacles]
            ) if obstacles else float('inf')

            # Penalize collision
            if min_obstacle_dist < self.config.obstacle_radius:
                return float('inf')

            trajectory_cost += distance_to_goal + (1.0 / min_obstacle_dist)

        return trajectory_cost

    def sample_velocities(self):
        """
        Generate velocity samples within dynamic constraints.
        """
        v_samples = np.arange(-self.config.max_linear_velocity,
                               self.config.max_linear_velocity, 0.05)
        w_samples = np.arange(-self.config.max_angular_velocity,
                               self.config.max_angular_velocity, 0.1)
        return [(v, w) for v in v_samples for w in w_samples]

    def plan(self, current_pose, goal_pose, obstacles):
        """
        Plan the best velocity using DWA.
        """
        best_cost = float('inf')
        best_velocity = (0, 0)
        for v_sample in self.sample_velocities():
            cost = self.compute_trajectory_cost(v_sample, goal_pose, current_pose, obstacles)
            if cost < best_cost:
                best_cost = cost
                best_velocity = v_sample

        return best_velocity


class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.current_pose = (0.0, 0.0, 0.0)
        self.goal_pose = (1.0, 1.0)
        self.obstacles = []
        self.dwa_planner = DWAPlanner(RobotConfig())

    def goal_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal received: {self.goal_pose}")

    def lidar_callback(self, msg):
        # Convert scan to obstacle positions
        self.obstacles = self.process_lidar(msg)

    def process_lidar(self, scan_data):
        """Process lidar data and return obstacle coordinates."""
        obstacles = []
        angle = scan_data.angle_min
        for distance in scan_data.ranges:
            if distance < scan_data.range_max:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                obstacles.append((x, y))
            angle += scan_data.angle_increment
        return obstacles

    def control_loop(self):
        velocity = self.dwa_planner.plan(self.current_pose, self.goal_pose, self.obstacles)
        self.publish_velocity(velocity)

    def publish_velocity(self, velocity):
        v, w = velocity
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = DWAController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
