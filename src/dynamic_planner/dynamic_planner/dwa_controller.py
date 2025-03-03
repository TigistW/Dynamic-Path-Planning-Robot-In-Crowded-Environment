import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.angle_min = 0.0  # Will be updated in scan_callback
        self.angle_increment = 0.0 

        # Robot parameters
        self.max_speed = 0.5  # m/s
        self.max_turn_rate = 1.0  # rad/s
        self.dt = 0.1  # Time step for velocity simulation
        self.obstacle_threshold = 0.5  # m (min distance to obstacles)

        self.goal = np.array([3.0, 2.0])  # Target position
        self.position = np.array([0.0, 0.0])  # Current position
        self.yaw = 0.0  # Robot heading
        self.lidar_ranges = None  # LiDAR scan data

        # Timer to periodically run the DWA control loop
        self.create_timer(0.1, self.dwa_control)

        self.get_logger().info("DWA Controller Node Started!")

    def odom_callback(self, msg):
        """Update robot position and orientation from odometry."""
        self.position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        orientation = msg.pose.pose.orientation
        self.yaw = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                              1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

    def scan_callback(self, msg):
        """ Store latest LiDAR scan data and detect obstacles. """
        if len(msg.ranges) == 0:
            self.get_logger().warn("Received empty LiDAR scan!")
            return

        self.lidar_ranges = np.array(msg.ranges)  # Store LiDAR ranges

       
        front_distances = self.lidar_ranges[np.r_[:30, -30:]]
        self.min_front_distance = np.min(front_distances) if len(front_distances) > 0 else float('inf')

        self.get_logger().info(f"Closest obstacle in front: {self.min_front_distance:.2f} meters")



    def dwa_control(self):
        """Main DWA algorithm: selects the best velocity for navigation."""
        if self.lidar_ranges is None:
            self.get_logger().warn("Waiting for LiDAR data...")
            return

        # Generate velocity samples
        velocities = np.linspace(-self.max_speed, self.max_speed, 5)
        angular_velocities = np.linspace(-self.max_turn_rate, self.max_turn_rate, 5)

        best_v = 0.0
        best_w = 0.0
        best_score = -np.inf

        for v in velocities:
            for w in angular_velocities:
                trajectory = self.simulate_trajectory(v, w)

                # Evaluate trajectory score components
                clearance = self.compute_clearance(trajectory)
                goal_score = self.compute_goal_score(trajectory[-1])
                speed_score = v  # Prefer higher speeds

                total_score = clearance * 1.5 + goal_score * 2.0 + speed_score

                if total_score > best_score:
                    best_score = total_score
                    best_v = v
                    best_w = w

        # Publish best velocity command
        twist_msg = Twist()
        twist_msg.linear.x = best_v
        twist_msg.angular.z = best_w
        self.get_logger().info(f"Evaluating v={best_v:.2f}, w={best_w:.2f}, Score={best_score:.2f}")
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Publishing Velocities: v={best_v:.2f}, w={best_w:.2f}")

    def simulate_trajectory(self, v, w):
        """Simulate the trajectory for given velocity inputs."""
        x, y, theta = self.position[0], self.position[1], self.yaw
        trajectory = []

        for _ in range(10):  # Simulate over a short horizon
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append((x, y))

        return np.array(trajectory)

    def compute_clearance(self, trajectory):
        """Compute clearance (distance to nearest obstacle along trajectory)."""
        min_distance = np.inf
        for x, y in trajectory:
            # Use the index to compute the correct angle for each LiDAR range value
            for i, lidar_range in enumerate(self.lidar_ranges):
                if lidar_range < self.obstacle_threshold:
                    actual_angle = self.angle_min + i * self.angle_increment
                    obs_x = self.position[0] + lidar_range * np.cos(actual_angle)
                    obs_y = self.position[1] + lidar_range * np.sin(actual_angle)
                    distance = np.linalg.norm([x - obs_x, y - obs_y])
                    min_distance = min(min_distance, distance)
        return min_distance

    def compute_goal_score(self, last_position):
        """Compute score based on how close the trajectory gets to the goal."""
        distance_to_goal = np.linalg.norm(self.goal - last_position)
        return 1.0 / (distance_to_goal + 1e-6)  # Avoid division by zero

def main(args=None):
    rclpy.init(args=args)
    node = DWAController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
