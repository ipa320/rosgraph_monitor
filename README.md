# rosgraph_monitor

## Installation
```
mkdir ~/model_ws && cd ~/model_ws
wget https://raw.githubusercontent.com/ipa-hsd/rosgraph_monitor/foxy-devel/.rosinstall
vcs import src < .rosinstall
source /opt/ros/foxy/setup.bash
rosdep update && rosdep install --from-paths src -y -i -r
colcon build
```
## Usage

### ROS system diagnostics
To generate rossystem model from the ROS graph and to compare it with a desired system, launch a system of choice:

Start the `rosgraph_monitor` and listen to `/diagnostics` topic  
(`source ~/model_ws/install/setup.bash` on each new terminal)
```
# Terminal 1:
ros2 run rosgraph_monitor graph_observer.py

#Terminal 2:
ros2 topic echo /diagnostics
```

### Property observer
To use a custom created property observer in the module `observers` (e.g. `QualityObserver`)
Note: Currently a dummy observer has been implemented in `observer.py`
```
# Terminal 1:
ros2 run rosgraph_monitor observer.py

#Terminal 2:
ros2 topic echo /diagnostics
```

