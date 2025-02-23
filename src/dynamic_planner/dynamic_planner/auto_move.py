import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutoMove(Node):
    def __init__(self):
        super().__init__('auto_move')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move_robot)  # Run every 1 sec

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)
        self.get_logger().info("Moving robot forward!")

def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
