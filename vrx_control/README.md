# vrx_control

## Overview

This package contains a node for controlling the course of a marine vehicle and for managing vehicle behaviours such as station keeping. The node responds to Course msgs (vrx_msgs/Course)
and sends appropriate commands to the boat's thrusters to create the appropriate behaviour.

## Usage

Run the course controller node individually with:

```roslaunch vrx_control course_controller.launch```

For a boat that supports the 'T' thruster configuration, use the following:

```roslaunch vrx_bringup sensored_boat.launch```

and launch the course controller with the thrust_config argument (see below).

To launch the whole wayfinding/station-keeping suite:

```roslaunch vrx_bringup sensored_boat.launch```
```roslaunch vrx_control control.launch```

## Config files

Configuration files are located in the config directory.

* **gazebo_wamv_course_controller.yaml** Settings for controlling the wamv in the Gazebo simulator.

## Launch files

* **course_controller.launch:** Launches the wamv course controller node.

	Optional Arguments:

	- **`thrust_config`** Specifies the thruster configuration for the vessel. Possibilities are 'H' for standard differential hull thrusters, and 'T' for an additional front lateral thruster. Note that station keeping is disabled for the 'H' configuration. Default: `'H'`.

	- **`overlying_config_file`** Provide a path to a different configuration file, e.g. `value="$(find vrx_control)/config/<your_config_file.yaml>"`. Default: `gazebo_wamv_course_controller.yaml`.

## Nodes

### wamv_course_controller

Receives course commands for the wamv and sends appropriate commands to thrusters.

#### Subscribed Topics

* **`/course_cmd`** ([vrx_msgs/Course])

	The desired course for the vehicle (i.e. speed and yaw), including optional station keeping information.

* **`/p3d_wamv`** ([nav_msgs/Odometry])

	The current wamv odometry.

#### Published Topics

* **`/right_thrust_cmd`** ([std_msgs/Float32])

	Thrust command for the right wamv thruster.

* **`/left_thrust_cmd`** ([std_msgs/Float32])

	Thrust command for the left wamv thruster.

* **`/lateral_thrust_cmd`** ([std_msgs/Float32])

	Thrust command for the lateral wamv thruster.

* **`/right_thrust_angle`** ([std_msgs/Float32])

	Changes the angle of the right wamv thruster.

#### Parameters

See configuration files in the config directory.
