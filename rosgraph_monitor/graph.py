import rclpy
from rclpy.node import Node
from rosgraph_monitor.parameters_helper import *
from re import compile


BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock", "/parameter_events", "/diagnostics"]
BLACK_LIST_SERV = ["describe_parameters", "get_parameter_types", "get_parameters", "list_parameters", "set_parameters", "set_parameters_atomically"]
BLACK_LIST_NODE = ["rosout", "_ros2cli_daemon_0"]

ACTION_FILTER = ['goal', 'cancel']
ACTION_FILTER2 = ['status', 'result', 'feedback']


def _init_node_dict(nodes, name):
    nodes[name] = {'parameters' : dict(),
                   'publishers' : dict(),
                   'subscribers' : dict(),
                   'service_servers' : dict(),
                   'service_clients' :dict(),
                   'action_servers' :dict(),
                   'action_clients' : dict() }

def _get_parameter_names_by_node(this_node: Node, node_name: str):
    param_names = call_list_parameters(this_node, node_name)
    response = call_get_parameters(this_node, node_name, param_names)
    param_values = [[get_value(x) for x in response.values]]
    parameters = dict(zip(param_names, param_values))
    return parameters

def create_ros_graph_snapshot(graph_node: Node):
    components = dict()

    nodes_ns = graph_node.get_node_names_and_namespaces()

    for node in nodes_ns:
        if node[0] in BLACK_LIST_NODE or node[0] == graph_node.get_name():
            continue

        if node not in components:
            node_name = node[1] + node[0]
            _init_node_dict(components, node_name)

        pubs = graph_node.get_publisher_names_and_types_by_node(node[0], node[1])
        for pub in pubs:
            if not pub[0].endswith(tuple(BLACK_LIST_TOPIC)):
                components[node_name]['publishers'][pub[0]] = pub[1][0]

        subs = graph_node.get_subscriber_names_and_types_by_node(node[0], node[1])
        for sub in subs:
            if not sub[0].endswith(tuple(BLACK_LIST_TOPIC)):
                components[node_name]['subscribers'][sub[0]] = sub[1][0]

        srvs = graph_node.get_service_names_and_types_by_node(node[0], node[1])
        for srv in srvs:
            if not srv[0].endswith(tuple(BLACK_LIST_SERV)):
                components[node_name]['service_servers'][srv[0]] = srv[1][0]

        parameters = _get_parameter_names_by_node(graph_node, node_name)
        components[node_name]['parameters'] = parameters

    return components
