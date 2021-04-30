import threading
# import mutex
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class Observer(object):
    def __init__(self, name, loop_rate_hz=1):
        self._name = name
        self._rate = rospy.Rate(loop_rate_hz)
        self._seq = 1
        self._lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run)
        self._thread.daemon = True
        self._stop_event = threading.Event()

        self._pub_diag = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)

    def __del__(self):
        if Observer:
            print("{} stopped".format(self._name))

    # Every derived class needs to override this
    def generate_diagnostics(self):
        msg = DiagnosticArray()
        return msg

    def _run(self):
        while not rospy.is_shutdown() and not self._stopped():
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = rospy.get_rostime()

            status_msgs = self.generate_diagnostics()
            diag_msg.status.extend(status_msgs)
            self._pub_diag.publish(diag_msg)

            self._seq += 1
            self._rate.sleep()

    def start(self):
        print("starting {}...".format(self._name))
        self._thread.start()

    def stop(self):
        self._lock.acquire()
        self._stop_event.set()
        self._lock.release()

    def _stopped(self):
        self._lock.acquire()
        isSet = self._stop_event.isSet()
        self._lock.release()
        return isSet


class ServiceObserver(Observer):
    def __init__(self, name, service_name=None, service_type=None, loop_rate_hz=1):
        super(ServiceObserver, self).__init__(name, loop_rate_hz)
        self.name = service_name
        self.type = service_type
        self.client = None
        try:
            self.start_service()
        except:
            print("{} service not started".format(self.name))
            super.__del__()

    def start_service(self):
        try:
            rospy.wait_for_service(self.name, timeout=1.0)
            self.client = rospy.ServiceProxy(self.name, self.type)
            print("Service '" + self.name +
                  "' added of type" + str(self.type))
        except rospy.ServiceException as exc:
            print("Service {} is not running: ".format(self.name) + str(exc))

    def generate_diagnostics(self):
        try:
            resp = self.client.call()
        except rospy.ServiceException as exc:
            print("Service {} did not process request: ".format(
                self.name) + str(exc))
        status_msg = self.diagnostics_from_response(resp)
        return status_msg

    # Every derived class needs to override this
    def diagnostics_from_response(self, response):
        msg = DiagnosticArray()
        return msg


class TopicObserver(Observer):
    def __init__(self, name, loop_rate_hz, topics):
        super(TopicObserver, self).__init__(name, loop_rate_hz)
        self._topics = topics
        self._id = ""
        self._num_topics = len(topics)

    # Every derived class needs to override this
    def calculate_attr(self, msgs):
        # do calculations
        return DiagnosticStatus()

    def generate_diagnostics(self):
        msgs = []
        received_all = True
        for topic, topic_type in self._topics:
            try:
                msgs.append(rospy.wait_for_message(topic, topic_type))
            except rospy.ROSException as exc:
                print("Topic {} is not found: ".format(topic) + str(exc))
                received_all = False
                break

        status_msgs = list()
        status_msg = DiagnosticStatus()
        if received_all:
            status_msg = self.calculate_attr(msgs)
        status_msgs.append(status_msg)

        return status_msgs
