# rosgraph_monitor

## Installation
```
mkdir ~/model_ws && cd ~/model_ws
wstool init src https://raw.githubusercontent.com/ipa320/rosgraph_monitor/main/.rosinstall
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update && rosdep install --from-paths ./src -y -i -r
catkin build
```
## Usage

### ROS system diagnostics
To generate rossystem model from the ROS graph and to compare it with a desired system, launch a system of choice:
```
# Terminal 1:
source ~/model_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
Start the `rosgraph_monitor` and listen to `/diagnostics` topic  
(`source ~/model_ws/devel/setup.bash` on each new terminal)
```
# Terminal 2:
rosrun rosgraph_monitor monitor

# Terminal 3:
rosservice call /load_observer "name: 'ROSGraphObserver'"

#Terminal 4:
rostopic echo /diagnostics
```
To dump the `rossystem` model of the running system started above in Terminal 1 to file `resources/test.rossystem`:
```
# Terminal 2:
rosrun rosgraph_monitor rossystem_snapshot
```
Observers can now be loaded at startup. Below is an example to load `ROSGraphObserver` and `DummyObserver`. `ROSGraphObserver` also needs an additional private parameter `_desired_rossystem` with absolute path of `rossystem` file.
```
# Terminal 2:
rosrun rosgraph_monitor monitor --load ROSGraphObserver DummyObserver _desired_rossystem:=<path/to/file>test.rossystem
```

### Property observer
To use a custom created property observer in the module `observers` (e.g. `QualityObserver`)

```
# Terminal 2:
rosrun rosgraph_monitor monitor

# Terminal 3:
rosservice call /load_observer "name: 'QualityObserver'"

#Terminal 4:
rostopic echo /diagnostics
```

