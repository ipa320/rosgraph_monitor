import threading
import typing as t

import rclpy
from rclpy.qos import QoSProfile, HistoryPolicy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Int32
from message_filters import ApproximateTimeSynchronizer, Subscriber

# TODO: not sure if it should extend Node
# Meaning, does every observer has to be a separate node?
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


class TopicObserver(Observer):
    def __init__(self, name, topics,
                loop_rate_hz=1,
                qos_profile=QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST)):
        super(TopicObserver, self).__init__(name, qos_profile, loop_rate_hz)

        subscribers = []
        for topic, topic_type in topics:
            sub = Subscriber(self, topic_type, topic)
            subscribers.append(sub)
        topic_synchronizer = ApproximateTimeSynchronizer(
            subscribers,
            5,  # queue size
            0.2,  # time
            allow_headerless=True)
        topic_synchronizer.registerCallback(self.callback)

    def callback(self, msg1, msg2):
        print(str(msg1) + " " + str(msg2))

    def generate_diagnostics(self) -> t.List[DiagnosticStatus]:
        msg = []
        msg.append(DiagnosticStatus())
        return msg

# TODO: delete later -- for test only
def main(args=None) -> None:
    rclpy.init(args=args)
    
    topics = [("/speed", Int32), ("/accel", Int32)]
    observer = TopicObserver("Dummy", topics)
    observer.start()
    try:
        rclpy.spin(observer)
    except KeyboardInterrupt:
        observer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()