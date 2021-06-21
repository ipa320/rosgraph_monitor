from rosgraph_monitor.observer import Observer
from ros_model_parser.rossystem_parser import RosSystemModelParser
from ros_model_generator.rossystem_generator import RosSystemModelGenerator
from rosgraph_monitor.graph import create_ros_graph_snapshot, get_param
from rosgraph_monitor.model import compare_rossystem_models
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ROSGraphObserver(Observer):
    def __init__(self, name):
        super(ROSGraphObserver, self).__init__(name)

        try:
            model_path = get_param('~desired_rossystem')
        except KeyError as exc:
            raise exc
        self.static_model = RosSystemModelParser(model_path).parse()

    def generate_diagnostics(self):
        status_msgs = list()

        components = create_ros_graph_snapshot()
        try:
            generator = RosSystemModelGenerator('demo')
            model_str = generator.create_ros_system_model_list(components)[1]
            dynamic_model = RosSystemModelParser(model_str, isFile=False).parse()
        except Exception as e:
            print(e.args)
            return status_msgs

        missing_interfaces, additional_interfaces, incorrect_params = compare_rossystem_models(
            self.static_model, dynamic_model)

        status_msgs = list()
        if (not missing_interfaces) & (not additional_interfaces) & (not incorrect_params):
            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.name = "ROS Graph"
            status_msg.message = "running OK"
            status_msgs.append(status_msg)

        else:
            # Here are 2 'for loops' - 1 for missing and 1 for additional
            for interface in missing_interfaces:
                status_msg = DiagnosticStatus()
                status_msg.level = DiagnosticStatus.ERROR
                status_msg.name = interface
                status_msg.message = "Missing node"
                status_msgs.append(status_msg)

            for interface in additional_interfaces:
                status_msg = DiagnosticStatus()
                status_msg.level = DiagnosticStatus.WARN
                status_msg.name = interface
                status_msg.message = "Additional node"
                status_msgs.append(status_msg)

            for interface in incorrect_params:
                status_msg = DiagnosticStatus()
                status_msg.level = DiagnosticStatus.ERROR
                status_msg.name = interface
                status_msg.message = "Wrong param configuration"
                for params in incorrect_params[interface]:
                    status_msg.values.append(
                        KeyValue(params[0], str(params[1])))
                status_msgs.append(status_msg)

        return status_msgs
