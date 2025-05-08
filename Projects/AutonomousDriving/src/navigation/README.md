# Content

This package contains the launch configuration of move_base and additional nodes required for the navigation task. The additional nodes are:

- master_node: Publishes waypoint poses as subgoals in order to follow the predefined track. The waypoints are specified in a text file in the resources folder. Interacts with the traffic_light_detector service to handle stopping at red lights.
- odom_rotator_node: Rotates odom message from the state_estimator_corruptor_node into the base_link frame of the vehicle.

# Usage

The navigation can be launched with the launch file:

```
roslaunch navigation move_base.launch
```

The waypoint publishing node can also be launched with a launch file:

```
roslaunch navigation master_node.launch
```