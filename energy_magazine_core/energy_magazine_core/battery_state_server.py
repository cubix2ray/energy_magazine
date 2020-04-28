import time
from matplotlib import pyplot
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
        self.last_callback_time = None
        self.state_of_charge = 0 # A*s
        self.max_capacity = 5*60*60 # A*s
        self.measurements = []
        # self.plot_time, self.plot_soc = [], []
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(self, BatteryState, 'battery_state',
                                           self.execute_callback,
                                           callback_group=callback_group)
        self.subscription = self.create_subscription(Float64, 'actual_current',
                                                     self.analysis_callback, 10,
                                                     callback_group=callback_group)
        self.get_logger().info('Starting battery state server')

    def analysis_callback(self, msg: Float64) -> None:
        now = time.time()
        current = float(msg.data)
        if self.last_callback_time is not None:
            time_passed, self.last_callback_time = now - self.last_callback_time, now
        else:
            time_passed, self.last_callback_time = 0, now
        self.state_of_charge += current*time_passed
        pyplot.plot(now, self.state_of_charge)
        pyplot.draw()
        self.get_logger().info('State of charge: ' + str(round(1000*self.state_of_charge, 2))
                               + 'mAh (' + str(round(self.state_of_charge/self.max_capacity, 1)) + '%)')
        if self.goal_handle is None:
            return
        if time.time() - self.start >= self.measure_time:
            self.goal_handle = None
            return

        self.measurements.append(current)
        feedback_msg = BatteryState.Feedback()
        feedback_msg.soc_percent = self.state_of_charge/self.max_capacity
        feedback_msg.soc_mah = self.state_of_charge * 1000 / 60 / 60
        feedback_msg.charge_state = feedback_msg.DISCHARGING if current < 0 else feedback_msg.CHARGING
        damage_limit = 10 if current < 0 else 5
        feedback_msg.battery_state = feedback_msg.OK if abs(current) < damage_limit else feedback_msg.DAMAGE
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
        mean_current = sum(self.measurements)/len(self.measurements)
        result.time_to_full = (self.max_capacity - self.state_of_charge)/mean_current
        self.measurements = []
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    pyplot.ion()
    pyplot.show()
    rclpy.spin(BatteryStateServer(), executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
