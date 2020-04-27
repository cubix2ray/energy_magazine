import time
import math as m

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from energy_magazine_msgs.action import BatteryState


class BatteryStateServer(Node):
    def __init__(self) -> None:
        super().__init__('battery_state_server')
        self.goal_handle = None
        self.measure_time = None
        self.start = None
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(self, BatteryState, 'battery_state',
                                           self.execute_callback,
                                           callback_group=callback_group)
        self.subscription = self.create_subscription(Float64, 'random_numbers',
                                                     self.analysis_callback, 10,
                                                     callback_group=callback_group)
        self.get_logger().info('Starting battery state server')

    def analysis_callback(self, msg: Float64) -> None:
        if self.goal_handle is None:
            return
        if time.time() - self.start >= self.measure_time:
            self.goal_handle = None
            return
        current = float(msg.data)

        feedback_msg = BatteryState.Feedback()
        self.goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle) -> None:
        self.get_logger().info('Executing goal...')
        self.goal_handle = goal_handle
        self.measure_time = self.goal_handle.request.time
        self.start = time.time()
        while self.goal_handle is not None:
            time.sleep(1)

        goal_handle.succeed()
        result = BatteryState.Result()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    rclpy.spin(BatteryStateServer(), executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
