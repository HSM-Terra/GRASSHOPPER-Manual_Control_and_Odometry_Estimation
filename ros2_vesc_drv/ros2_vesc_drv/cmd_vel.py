import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # Example linear velocity
        msg.angular.z = 0.0  # Example angular velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel: Linear - %.2f, Angular - %.2f' % (msg.linear.x, msg.angular.z))
        self.counter += 1
        if self.counter >= 300:  # 5 seconds at 10 Hz
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()