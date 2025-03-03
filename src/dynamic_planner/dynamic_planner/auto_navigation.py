import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AutoNavigation(Node):
    def __init__(self):
        super().__init__('auto_navigation')

        # Create publisher for movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to LiDAR scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # Control variables
        self.safe_distance = 0.5  # If an obstacle is closer than 0.5 meters, turn
        self.moving_forward = True
        self.get_logger().info("AutoNavigation Node Started - Moving Forward")

    def lidar_callback(self, msg):
        """ Process LiDAR data and decide movement. """
        front_distance = min(min(msg.ranges[0:30] + msg.ranges[-30:]), 10)  # Front center
        self.get_logger().info(f"Front obstacle distance: {front_distance:.2f} meters")

        twist_msg = Twist()

        if front_distance < self.safe_distance:
            self.get_logger().warn("Obstacle detected! Changing direction...")
            twist_msg.linear.x = 0.0  # Stop forward movement
            twist_msg.angular.z = 0.5  # Rotate in place
            self.moving_forward = False
        else:
            self.get_logger().info("Path clear! Moving forward...")
            twist_msg.linear.x = 0.5  # Move forward
            twist_msg.angular.z = 0.0  # No rotation
            self.moving_forward = True

        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
