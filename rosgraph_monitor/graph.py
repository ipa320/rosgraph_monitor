import rclpy
from rclpy.node import Node
from rosgraph_monitor.parameters_helper import *
from re import compile


BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock", "/parameter_events", "/diagnostics"]
BLACK_LIST_SERV = ["describe_parameters", "get_parameter_types", "get_parameters", "list_parameters", "set_parameters", "set_parameters_atomically"]
BLACK_LIST_NODE = ["rosout", "_ros2cli_daemon_0"]

ACTION_FILTER = ['send_goal', 'cancel_goal', 'get_result']
ACTION_FILTER2 = ['status', 'feedback']


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
    param_values = [get_value_and_type(x) for x in response.values]
    parameters = dict(zip(param_names, param_values))
    return parameters

def _check_actions(publishers, subscribers, services, action_clients, action_servers):
    pubs_ = [pub for pub in publishers.keys()]
    subs_ = [sub for sub in subscribers.keys()]
    srvs_ = [srv for srv in services.keys()]

    # Check Action Server
    for srv in srvs_:
        if (ACTION_FILTER[0] in srv) or (ACTION_FILTER[1] in srv) or (ACTION_FILTER[2] in srv) and '/_action/' in srv:
            action_name = '/' + srv.split('/')[1]
            if action_name not in action_servers.keys() and services[srv].endswith('_SendGoal'):
                if any(action_name in pub for pub in pubs_):
                    action_servers[action_name] = services[srv].partition('_SendGoal')[0]
            services.pop(srv)

    for pub in pubs_:
        if '/_action/' in pub:
            publishers.pop(pub)
    for sub in subs_:
        if '/_action/' in sub:
            subscribers.pop(sub)

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

        # the service 'list_parameters' times out causing a runtime exception
        # will debug the issue later
        # parameters = _get_parameter_names_by_node(graph_node, node_name)
        # components[node_name]['parameters'] = parameters

        publishers = components[node_name]['publishers']
        subscribers = components[node_name]['subscribers']
        services = components[node_name]['service_servers']
        action_clients = components[node_name]['action_clients']
        action_servers = components[node_name]['action_servers']
        _check_actions(publishers, subscribers, services, action_clients, action_servers)

    return components
