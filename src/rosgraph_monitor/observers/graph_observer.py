from rosgraph_monitor.observer import Observer
from ros_model_parser.rossystem_parser import RosSystemModelParser
from ros_model_generator.rossystem_generator import RosSystemModelGenerator
import rosgraph
import rosparam
import rospkg
import rosservice
import imp
import os.path
from re import compile

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock"]
BLACK_LIST_SERV = ["/set_logger_level", "/get_loggers"]
BLACK_LIST_NODE = ["/rosout"]

ACTION_FILTER = ['goal', 'cancel']
ACTION_FILTER2 = ['status', 'result', 'feedback']

def strip_slash(string):
    return '{}'.format(string[1:] if string.startswith('/') else string)

def check_black_list(name, black_list):
    for bl_ in black_list:
        if bl_ in name:
            return False
    return True

def init_node_dict(nodes, name):
    nodes[name] = {'parameters' : dict(),
                   'publishers' : dict(),
                   'subscribers' : dict(),
                   'service_servers' : dict(),
                   'service_clients' :dict(),
                   'action_servers' :dict(),
                   'action_clients' : dict() }

def check_actions(publishers, subscribers, action_clients, action_servers):
        pubs_ = [pub for pub in publishers.keys()]
        subs_ = [sub for sub in subscribers.keys()]

        remove_pubs = list()
        remove_subs = list()

        # Check Action client
        for topic_name, topic_type in publishers.items():
            if topic_name.endswith(ACTION_FILTER[0]):
                _action_name = topic_name[:-len(ACTION_FILTER[0]) - 1]
                cancel_topic = _action_name + '/' + ACTION_FILTER[1]
                if not (cancel_topic in pubs_):
                    continue
                remove_pubs.append(topic_name)
                remove_pubs.append(cancel_topic)
                for name in ACTION_FILTER2:
                    topic = _action_name + '/' + name
                    if not (topic in subs_):
                        continue
                    remove_subs.append(topic)
                _action_type = topic_type[:-10]  # Hardcoded ActionGoal
                action_clients.add((_action_name, _action_type))

        # Check Action Server
        for topic_name, topic_type in subscribers.items():
            if topic_name.endswith(ACTION_FILTER[0]):
                _action_name = topic_name[:-len(ACTION_FILTER[0]) - 1]
                cancel_topic = _action_name + '/' + ACTION_FILTER[1]
                if not (cancel_topic in subs_):
                    continue
                remove_subs.append(topic_name)
                remove_subs.append(cancel_topic)
                for name in ACTION_FILTER2:
                    topic = _action_name + '/' + name
                    if not (topic in pubs_):
                        continue
                    remove_pubs.append(topic)
                _action_type = topic_type[:-10]  # Hardcode ActionGoal
                action_servers[_action_name] = _action_type

        for topic in remove_pubs:
            publishers.pop(topic)
        for topic in remove_subs:
            subscribers.pop(topic)

def create_ros_graph_snapshot():
    master = rosgraph.Master('snapshot')
    params = list()
    topics_dict = dict()

    if not(master.is_online()):
        print("Error: ROSMaster not found")
        return list()

    state = master.getSystemState() #get the system state
    pubs, subs, services = state

    #get all topics type
    topic_list = master.getTopicTypes()
    for topic, topic_type in topic_list:
        topics_dict[topic] = topic_type

    components = dict()
    for pub, nodes in pubs:
        if not check_black_list(pub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['publishers'][pub] = topics_dict[pub]

    for sub, nodes in subs:
        if not check_black_list(sub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['subscribers'][sub] = topics_dict[sub]

    for serv, nodes in services:
        if not check_black_list(serv, BLACK_LIST_SERV):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['service_servers'][serv] = rosservice.get_service_type(serv)

    for name in components:
        publishers = components[name]['publishers']
        subscribers = components[name]['subscribers']
        action_clients = components[name]['action_clients']
        action_servers = components[name]['action_servers']
        check_actions(publishers, subscribers, action_clients, action_servers)

    # Get parameters
    params = master.getParamNames()
    component_names = list(components.keys())
    for name in component_names:
        r = compile(name + "*")
        # Python2.x: param_node_ns = filter(r.match, params)
        param_node_ns = list(filter(r.match, params))
        # remove the params which belong to the node's namespace from the list
        # the params that remain at the end of the loop are global params
        g_params = [param for param in params if param not in param_node_ns]
        params = g_params
        for param in param_node_ns:
            if param not in BLACK_LIST_PARAM and not(param.startswith('/roslaunch')):
                p = master.getParam(param)
                components[node]['parameters'][param] = [p, type(p)]
    # the remaining params are global params
    if len(params) > 0:
        init_node_dict(components, 'parameters_node')
        for param in params:
            if param not in BLACK_LIST_PARAM and not(param.startswith('/roslaunch')):
                p = master.getParam(param)
                components['parameters_node']['parameters'][param] = [p, type(p)]

    return components


class ROSGraphObserver(Observer):
    def __init__(self, name):
        super(ROSGraphObserver, self).__init__(name)
        rospack = rospkg.RosPack()
        # TODO: path to model shouldn't be hardcoded
        model_path = os.path.join(rospack.get_path('rosgraph_monitor'), "resources/cob4-25.rossystem")
        self.static_model = RosSystemModelParser(model_path).parse()
        self.generator = RosSystemModelGenerator('demo')

    def generate_diagnostics(self):
        status_msgs = list()

        components = create_ros_graph_snapshot()
        try:
            model_str = self.generator.create_ros_system_model_list(components)[1]
            dynamic_model = RosSystemModelParser(model_str, isFile=False).parse()
        except Exception as e:
            print(e.args)
            return status_msgs

        missing_interfaces, additional_interfaces, incorrect_params = self.compare_models(
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

    # find out missing and additional interfaces
    # if both lists are empty, system is running fine
    def compare_models(self, model_ref, model_current):
        # not sure of the performance of this method
        set_ref = set((strip_slash(x.interface_name[0]))
                      for x in model_ref.interfaces)
        set_current = set((strip_slash(x.interface_name[0]))
                          for x in model_current.interfaces)

        # similarly for all interfaces within the node?
        # or only for topic connections?
        # does LED's code capture topic connections?
        ref_params = dict()
        for interface in model_ref.interfaces:
            for param in interface.parameters:
                key = strip_slash(param.param_name[0])
                ref_params[key] = [param.param_value[0],
                                   interface.interface_name[0]]

        current_params = dict()
        for interface in model_current.interfaces:
            for param in interface.parameters:
                key = strip_slash(param.param_name[0])
                current_params[key] = [
                    param.param_value[0], interface.interface_name[0]]

        incorrect_params = dict()
        for key, value in ref_params.items():
            try:
                current_value = current_params[key][0]
                ref_value = ref_params[key][0]

                if (type(current_value) is ParseResults) & (type(ref_value) is ParseResults):
                    current_value = current_value.asList()
                    ref_value = ref_value.asList()
                if (type(current_value) is str) & (type(ref_value) is str):
                    current_value = re.sub(
                        r"[\n\t\s]*", "", strip_slash(current_value))
                    ref_value = re.sub(
                        r"[\n\t\s]*", "", strip_slash(ref_value))
                isEqual = current_value == ref_value
                if not isEqual:
                    incorrect_params.setdefault(current_params[key][1], [])
                    incorrect_params[current_params[key]
                                     [1]].append([key, current_value])
            except Exception as exc:
                pass

        # returning missing_interfaces, additional_interfaces
        return list(set_ref - set_current), list(set_current - set_ref), incorrect_params
