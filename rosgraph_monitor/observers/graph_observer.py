from rosgraph_monitor.observer import Observer
# from ros_model_parser.rossystem_parser import RosSystemModelParser
# from ros_model_generator.rossystem_generator import RosSystemModelGenerator
from rosgraph_monitor.graph import create_ros_graph_snapshot
# from rosgraph_monitor.model import compare_rossystem_models
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# to be deleted
import rclpy


class ROSGraphObserver(Observer):
    def __init__(self, name):
        super(ROSGraphObserver, self).__init__(name)

    def generate_diagnostics(self):
        components = create_ros_graph_snapshot(self)
        return components
        

# TODO: delete later -- for test only
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ROSGraphObserver('rosgraph_observer')

    try:
        while rclpy.ok():
            print(node.generate_diagnostics())
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()