#!/usr/bin/env python3
#
# Copyright 2021 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from rosgraph_monitor.observer import Observer
from ros_model_parser.ros_model_parser.rossystem_parser import RosSystemModelParser
from ros_model_parser.ros_model_generator.rossystem_generator import RosSystemModelGenerator
from rosgraph_monitor.graph import create_ros_graph_snapshot
from rosgraph_monitor.model import compare_rossystem_models, get_status_msgs

# to be deleted
import rclpy
import os


class ROSGraphObserver(Observer):
    def __init__(self, name):
        super(ROSGraphObserver, self).__init__(name)

        this_path = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(this_path, "../../resources/test.rossystem")
        self.static_model = RosSystemModelParser(model_path).parse()

    def generate_diagnostics(self):
        status_msgs = list()

        components = create_ros_graph_snapshot(self)
        try:
            generator = RosSystemModelGenerator('demo')
            model_str = generator.create_ros_system_model_list(components)[1]
            dynamic_model = RosSystemModelParser(model_str, isFile=False).parse()
        except Exception as e:
            print(e.args)
            return status_msgs

        missing_interfaces, additional_interfaces, incorrect_params = compare_rossystem_models(
            self.static_model, dynamic_model)

        status_msgs = get_status_msgs(missing_interfaces, additional_interfaces, incorrect_params)
        return status_msgs


# TODO: delete later -- for test only
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ROSGraphObserver('rosgraph_observer')
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
