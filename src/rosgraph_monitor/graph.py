import rosgraph
import rosparam
import rospkg
import rospy
import rosservice
from re import compile


BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock"]
BLACK_LIST_SERV = ["/set_logger_level", "/get_loggers"]
BLACK_LIST_NODE = ["/rosout"]

ACTION_FILTER = ['goal', 'cancel']
ACTION_FILTER2 = ['status', 'result', 'feedback']


def get_param(param_name):
    if not rospy.has_param('~desired_rossystem'):
        raise KeyError("Private parameter 'desired_rossystem' not set")
    return rospy.get_param('~desired_rossystem')

def _check_black_list(name, black_list):
    for bl_ in black_list:
        if bl_ in name:
            return False
    return True

def _init_node_dict(nodes, name):
    nodes[name] = {'parameters' : dict(),
                   'publishers' : dict(),
                   'subscribers' : dict(),
                   'service_servers' : dict(),
                   'service_clients' :dict(),
                   'action_servers' :dict(),
                   'action_clients' : dict() }

def _check_actions(publishers, subscribers, action_clients, action_servers):
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
                if _action_name not in action_clients.keys():
                    action_clients[_action_name] = [_action_type]
                else:
                    action_clients[_action_name].append(_action_type)

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
        if not _check_black_list(pub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not _check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                _init_node_dict(components, node)
            components[node]['publishers'][pub] = topics_dict[pub]

    for sub, nodes in subs:
        if not _check_black_list(sub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not _check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                _init_node_dict(components, node)
            components[node]['subscribers'][sub] = topics_dict[sub]

    for serv, nodes in services:
        if not _check_black_list(serv, BLACK_LIST_SERV):
            continue
        for node in nodes:
            if not _check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                _init_node_dict(components, node)
            try:
                components[node]['service_servers'][serv] = rosservice.get_service_type(serv)
            except rosservice.ROSServiceIOException as e:
                pass

    for name in components:
        publishers = components[name]['publishers']
        subscribers = components[name]['subscribers']
        action_clients = components[name]['action_clients']
        action_servers = components[name]['action_servers']
        _check_actions(publishers, subscribers, action_clients, action_servers)

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
                components[name]['parameters'][param] = [p, type(p)]
    # the remaining params are global params
    if len(params) > 0:
        components['global_parameters'] = dict()
        for param in params:
            if param not in BLACK_LIST_PARAM and not(param.startswith('/roslaunch')):
                p = master.getParam(param)
                components['global_parameters'][param] = [p, type(p)]

    return components
