from pyparsing import *
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import re


def strip_slash(string):
    return '{}'.format(string[1:] if string.startswith('/') else string)

# find out missing and additional interfaces
# if both lists are empty, system is running fine
def compare_rossystem_models(model_ref, model_current):
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
        except Exception:
            pass

    # returning missing_interfaces, additional_interfaces
    return list(set_ref - set_current), list(set_current - set_ref), incorrect_params


def get_status_msgs(missing_interfaces, additional_interfaces, incorrect_params):
    status_msgs = list()
    if (not missing_interfaces) & (not missing_interfaces) & (not missing_interfaces):
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
