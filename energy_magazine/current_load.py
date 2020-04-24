import rclpy
from rclpy.node import Node


class CurrentLoad(Node):
    def __init__(self):
        super().__init__('current_load')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CurrentLoad())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
