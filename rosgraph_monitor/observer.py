import threading
import typing as t

import rclpy
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# TODO: not sure if it should extend Node
class Observer(Node):
    def __init__(self, name, 
                qos_profile=QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST),
                loop_rate_hz=1):
        super(Observer, self).__init__(name)
        self._rate = self.create_rate(loop_rate_hz)
        self._logger = self.get_logger()
        self._clock = self.get_clock()

        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._stop_event = threading.Event()

        self._pub_diag = self.create_publisher(
            DiagnosticArray, 'diagnostics', qos_profile)

    def __del__(self):
        if Observer:
            self._logger.info("{} stopped".format(self.get_name()))

    # Every derived class needs to override this
    def generate_diagnostics(self) -> t.List[DiagnosticStatus]:
        msg = []
        msg.append(DiagnosticStatus())
        return msg

    def _run(self) -> None:
        self._logger.info("starting loop")
        while rclpy.ok() and not self._stopped():
            diag_msg = DiagnosticArray()
            
            status_msgs = self.generate_diagnostics()
            diag_msg.status.extend(status_msgs)

            diag_msg.header.stamp = self._clock.now().to_msg()
            self._pub_diag.publish(diag_msg)

            self._rate.sleep()

    def start(self) -> None:
        self._logger.info("starting {}...".format(self.get_name()))
        self._thread.start()

    def stop(self) -> None:
        self._lock.acquire()
        self._stop_event.set()
        self._lock.release()

    def _stopped(self) -> bool:
        self._lock.acquire()
        isSet = self._stop_event.isSet()
        self._lock.release()
        return isSet

# TODO: delete later -- for test only
def main(args=None) -> None:
    rclpy.init(args=args)
    
    observer = Observer("Dummy")
    observer.start()
    rclpy.spin(observer)

if __name__ == '__main__':
    main()