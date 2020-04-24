from os.path import join
from time import sleep
import threading
import rclpy
from rclpy.node import Node
import scipy.io as spio
from ament_index_python import get_package_share_directory
from std_msgs.msg import Float64


class CurrentLoad(Node):
    # węzeł odpowiadający za publikowanie przebiegu prądu
    def __init__(self, index=0):
        super().__init__('current_load')
        # wywołanie metody odczytującej dane z pliku .mat
        time, data = self.parse_matlab_data(index)
        self.publisher = self.create_publisher(Float64, 'actual_current', 10)
        # uruchomienie wątku z publikowaniem danych
        threading.Thread(target=self.publish_current, args=(time, data)).start()
        self.get_logger().info('Publishing data from index ' + str(index))

    def parse_matlab_data(self, index):
        # konstrukcja ścieżki do pliku .mat
        matlab_data_path = join(get_package_share_directory('energy_magazine'),
                                'matlab', 'B0007.mat')
        # otwarcie pliku .mat
        battery_data = spio.loadmat(matlab_data_path,
                                    squeeze_me=True)['B0007']['cycle'].item()['data']
        # odczytanie przebiegu prądu
        current = list(battery_data[index]['Current_measured'].item())
        # odczytanie wektora czasu
        time = list(battery_data[index]['Time'].item())
        # obliczenie różnic pomiędzy próbkami
        time_diffs = [current - previous for current, previous in zip(time[1:], time[:-1])]
        self.get_logger().info('Parsed matlab data of len ' + str(len(current)))
        return time_diffs, current

    def publish_current(self, time, data):
        # ta metoda publikuje wartości z przebiegu
        for time_to_wait, value in zip(time, data):
            # stworzenie i opublikowanie wiadomości z kolejną próbką
            msg = Float64()
            msg.data = value
            self.publisher.publish(msg)
            self.get_logger().info('Publishing current: ' + str(msg.data) + ' A')
            # odczekanie różnicy pomiędzy kolejnymi próbkami
            sleep(time_to_wait)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CurrentLoad())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
