import rclpy
from rclpy.node import Node
from re import compile


BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock"]
BLACK_LIST_SERV = ["/set_logger_level", "/get_loggers"]
BLACK_LIST_NODE = ["/rosout"]

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

def create_ros_graph_snapshot(graph_node: Node):
    components = dict()

    nodes_ns = graph_node.get_node_names_and_namespaces()

    for node in nodes_ns:
        if node not in components:
            node_name = node[1] + node[0]
            _init_node_dict(components, node_name)

        pubs = graph_node.get_publisher_names_and_types_by_node(node[0], node[1])
        for pub in pubs:
            components[node_name]['publishers'][pub[0]] = pub[1][0]

        subs = graph_node.get_subscriber_names_and_types_by_node(node[0], node[1])
        for sub in subs:
            components[node_name]['subscribers'][sub[0]] = sub[1][0]

        srvs = graph_node.get_service_names_and_types_by_node(node[0], node[1])
        for srv in srvs:
            components[node_name]['service_servers'][srv[0]] = srv[1][0]

        # TODO: for parameters
        # https://github.com/ros2/ros2cli/blob/c00dec0a72c049d3a4a8a80f1324ea24dc8373c6/ros2param/ros2param/api/__init__.py#L122
        # https://answers.ros.org/question/340600/how-to-get-ros2-parameter-hosted-by-another-node/

    return components
