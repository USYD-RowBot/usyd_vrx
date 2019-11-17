# vrx_waypoints

## Overview

This package contains a node for commanding a vessel to follow a waypoint route. The
node receives a WaypointRoute msg (vrx_msgs/WaypointRoute), produces Course msg commands
for the vessel (vrx_msgs/Course) in order to command its movement along the route.

## Usage

To run solely the waypoint follower node, use:

```roslaunch vrx_waypoints waypoint_follower.launch```

To test the whole system with waypoint following, instead run:

```roslaunch vrx_bringup sensored_boat.launch```

```roslaunch vrx_control control.launch```

To test station keeping, publish a vrx_msgs/WaypointRoute message in the terminal, e.g.:

```
rostopic pub /waypoints_cmd vrx_msgs/WaypointRoute "waypoints:
- nav_type: 1
  pose:
    position:
      x: 140.0
      y: 120.0
    orientation:
      x: 0.0
      y: 0.0
      z: 2.0
      w: 1.0
  station_duration: -1.0
speed: 2.0"
```

The 'station_duration' field indicates how long the vessel should remain at a station after aligning the correct pose, with a negative duration meaning the vessel should remain there indefinitely until receiving a new waypoint route. The 'nav_type' field uses a constant defined in the msg file (essentially an enumerator) to specify if the coordinate is a waypoint (for simply passing through) or a station to be kept with a required pose and duration. 

For more information on the WaypointRoute messages, see the definition for the vrx_msgs/WaypointRoute and vrx_msgs/Waypoint messages in usyd_vrx/vrx_msgs/msg. 

## Config files

Configuration files are located in the config directory.

* **gazebo_wamv_waypoint_follower.yaml** Settings for waypoint following in the Gazebo simulator.

## Launch files

* **waypoint_follower.launch:** Launches the waypoint following node.

     Optional Arguments:

     - **`overlying_config_file`** Provide a path to a different configuration file, e.g. `value="$(find vrx_waypoints)/config/<your_config_file.yaml>"`. Default: `gazebo_wamv_waypoint_follower.yaml`.

## Nodes

### wamv_waypoint_follower

Receives waypoint route requests for the wamv and sends appropriate commands to the course controller node.

#### Subscribed Topics

* **`/waypoints_cmd`** ([vrx_msgs/WaypointRoute])

	The desired route for the vehicle to follow, including waypoints and/or stations.

* **`/p3d_wamv`** ([nav_msgs/Odometry])

	The current wamv odometry.

#### Published Topics

* **`/course_cmd`** ([vrx_msgs/Course])

	The desired course command for the vehicle (i.e. speed and yaw), including optional station keeping information.

* **`/request_waypoints`** ([std_msgs/Empty])

	Requests a new waypoint route when the currently loaded route has been completed.

#### Parameters

See configuration files in the config directory.
