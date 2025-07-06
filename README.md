# PROLAB

**PROLAB**: Implementation of localization algorithms — Kalman Filter (KF), Extended Kalman Filter (EKF), and Particle Filter.

## Getting Started

Clone the repository:

```bash
git clone https://github.com/KRottonara/PROLAB.git prolab_ws
```

Build the workspace:

```bash
catkin_make
```

Start the first filter:

```bash
roslaunch example_package start_with_navigation.launch
```

## Nodes in This Workspace

- `kalman_filter_node`
- `extended_kalman_filter_node`
- `particle_filter_node`

You can switch between filters by commenting or uncommenting the relevant node in the launch file:

```xml
<!-- <node
    pkg="example_package"
    type="kalman_filter_node"
    name="kalman_filter_node"
    launch-prefix="xterm -e"/> -->
<!-- <node
    pkg="example_package"
    type="extended_kalman_filter_node"
    name="extended_kalman_filter_node"
    launch-prefix="xterm -e"/> -->
<node
    pkg="example_package"
    type="particle_filter_node"
    name="particle_filter_node"
    launch-prefix="xterm -e"/>
```

## Dynamic Reconfiguration

Dynamic reconfiguration is available for all nodes except the particle filter (you can enable it by uncommenting the relevant code).

To use dynamic reconfiguration:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Running the TurtleBot

The TurtleBot will automatically drive around the map in RViz. To make it drive in a smaller circle, comment or uncomment the relevant lines in:

```
src/example_package/config/goals.yaml
```
