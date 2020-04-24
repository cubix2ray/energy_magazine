from os.path import join
from time import sleep
import threading
import rclpy
from rclpy.node import Node
import scipy.io as spio
from ament_index_python import get_package_share_directory
from std_msgs.msg import Float64


class CurrentLoad(Node):
    def __init__(self, discharge=False):
        super().__init__('current_load')
        time, data = self.parse_matlab_data(discharge)
        self.publisher = self.create_publisher(Float64, 'actual_current', 10)
        threading.Thread(target=self.publish_current, args=(time, data)).start()
        mode = 'discharge' if discharge else 'charge'
        self.get_logger().info('Publishing ' + mode + ' data')

    def parse_matlab_data(self, discharge):
        index = 1 if discharge else 0
        matlab_data_path = join(get_package_share_directory('energy_magazine'),
                                'matlab', 'B0007.mat')
        battery_data = spio.loadmat(matlab_data_path,
                                    squeeze_me=True)['B0007']['cycle'].item()['data']
        current = list(battery_data[index]['Current_measured'].item())
        time = list(battery_data[index]['Time'].item())
        time_diffs = [current - previous for current, previous in zip(time[1:], time[:-1])]
        self.get_logger().info('Parsed matlab data of len ' + str(len(current)))
        return time_diffs, current

    def publish_current(self, time, data):
        for time_to_wait, value in zip(time, data):
            msg = Float64()
            msg.data = value
            self.publisher.publish(msg)
            self.get_logger().info('Publishing current: ' + str(msg.data) + ' A')
            sleep(time_to_wait)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CurrentLoad())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
