import time
import rclpy
import csv
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from energy_magazine_msgs.action import BatteryState


class BatteryStateServer(Node):
    # węzeł odpowiedzialny za symulację magazynu energii
    def __init__(self) -> None:
        super().__init__('battery_state_server')
        # tworzenie zmiennych potrzebnych do obliczania SOC i time_to_full
        self.goal_handle = None
        self.measure_time = None
        self.start = None
        self.last_callback_time = None
        self.state_of_charge = 0 # A*s
        self.max_capacity = 5*60*60 # A*s
        self.measurements = []
        # listy potrzebne do logowania danych
        self.log_time, self.log_soc, self.log_current = [], [], []
        # tworzymy callback grupę żeby możliwe było równoczesne przetwarzanie callbacków
        callback_group = ReentrantCallbackGroup()
        # tworzymy serwer akcji obliczający stan baterii
        self._action_server = ActionServer(self, BatteryState, 'battery_state',
                                           self.execute_callback,
                                           callback_group=callback_group)
        # tworzymy subscribera do tematu z obciążeniem prądowym
        self.subscription = self.create_subscription(Float64, 'current',
                                                     self.analysis_callback, 10,
                                                     callback_group=callback_group)
        self.get_logger().info('Starting battery state server')

    # ten callback uruchamiany jest przy każdej wiadomośći o obciążeniu prądowym
    def analysis_callback(self, msg: Float64) -> None:
        # sprawdzamy czas
        now = time.time()
        current = float(msg.data)
        # jeżeli jest to pierwsze wywołanie to odpowiednio inicjujemy zmienne
        if self.last_callback_time is None:
            time_passed, self.last_callback_time = 0, now
            self.log_time.append(0)
        else:
            time_passed, self.last_callback_time = now - self.last_callback_time, now
            self.log_time.append(self.log_time[-1]+time_passed)
        # obliczamy SOC jako całkę z natężenia prądu
        self.state_of_charge += current*time_passed
        # logujemy obliczone wartości
        self.log_soc.append(self.state_of_charge)
        self.log_current.append(current)
        self.get_logger().info('State of charge: ' + str(round(1000*self.state_of_charge, 2)) +
                               'mAh (' + str(round(self.state_of_charge/self.max_capacity, 1)) +
                               '%)')
        # jeżeli nie obsługujemy klienta, to wychodzimy z funkcji
        if self.goal_handle is None:
            return
        # jeżeli przekroczymy czas obsługi klienta to wychodzimy z funkcji
        if time.time() - self.start >= self.measure_time:
            self.goal_handle = None
            return
        # jeżeli obsługujemy klienta to dopisujemy prąd do pomiarów
        self.measurements.append(current)
        feedback_msg = BatteryState.Feedback()
        feedback_msg.soc_percent = self.state_of_charge/self.max_capacity
        feedback_msg.soc_mah = self.state_of_charge * 1000 / 60 / 60
        # określamy stan jako rozładowywanie jeżeli prąd jest ujemny, w przeciwnym razie jest to ładowanie
        feedback_msg.charge_state = feedback_msg.DISCHARGING if current < 0 else feedback_msg.CHARGING
        # określamy maksymalny dopuszczalny prąd w zależności od ładowania/rozładowywania
        damage_limit = 10 if current < 0 else 5
        # stan baterii jest OK jeżeli nie ma za dużego prądu, w przeciwnym razie bateria jest uszkodzona
        feedback_msg.battery_state = feedback_msg.OK if abs(current) < damage_limit else feedback_msg.DAMAGE
        # publikujemy informację zwrotną do klienta
        self.goal_handle.publish_feedback(feedback_msg)

    # ten callback jest wywołany kiedy przyjdzie zapytanie od klienta
    def execute_callback(self, goal_handle) -> None:
        self.get_logger().info('Executing goal...')
        self.goal_handle = goal_handle
        self.measure_time = self.goal_handle.request.time
        self.start = time.time()
        # czekamy aż serwer przetworzy zapytanie
        while self.goal_handle is not None:
            time.sleep(1)
        #zapisujemy dane
        self.save_data('current_and_soc', (self.log_time, self.log_current, self.log_soc))
        goal_handle.succeed()
        # wyliczamy przewidywany czas ładowania na podstawie średniego przepływu ładunku
        result = BatteryState.Result()
        mean_current = sum(self.measurements)/len(self.measurements)
        result.time_to_full = (self.max_capacity - self.state_of_charge)/mean_current
        self.measurements = []
        return result

    def save_data(self, file_name: str, data: tuple) -> None:
        # tworzymy nowy plik o podanej nazwie i zapisujemy każdy element krotki
        with open(file_name + '.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerows(data)


def main(args=None) -> None:
    rclpy.init(args=args)
    # używamy wielowątkowego executora żeby callbacki mogły wykonywać się równocześnie
    executor = MultiThreadedExecutor()
    rclpy.spin(BatteryStateServer(), executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
