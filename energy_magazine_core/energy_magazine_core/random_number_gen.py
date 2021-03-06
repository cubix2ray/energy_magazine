from random import uniform
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class RandomNumberGen(Node):
    def __init__(self):
        super().__init__('random_number_generator')
        self.publisher_ = self.create_publisher(Float64, 'random_numbers', 10)
        timer_period = 1 #s
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = uniform(-8, 13)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: ' + str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomNumberGen())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
