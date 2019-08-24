# vrx_control_2

## Overview

This package contains nodes for controlling the course of a marine vehicle and for managing vehicle behaviours such as waypoint following and station keeping. The intended application is for the Virtual RobotX challenge.

## Usage

Run the course controller node with:

	roslaunch vrx_control course_controller.launch

## Config files

Configuration files are located in the config directory.

* **gazebo_wamv.yaml** Settings for controlling the wamv in the Gazebo simulator.

## Launch files

* **course_controller.launch:** Launches the wamv course controller node.

     Optional Arguments:

     - **`overlying_config_file`** Provide a path to a different configuration file, e.g. `value="$(find vrx_control)/config/<your_config_file.yaml>"`. Default: `gazebo_wamv.yaml`.

## Nodes

### wamv_course_controller

Receives course commands for the wamv and sends appropriate commands to thrusters.

#### Subscribed Topics

* **`/cmd_course`** ([vrx_msgs/Course])

	The desired course for the vehicle (i.e. speed and yaw).

* **`/p3d_wamv`** ([nav_msgs/Odometry])

	The current wamv odometry.

#### Published Topics

* **`/right_thrust_cmd`** ([std_msgs/Float32])

	Thrust command for the right wamv thruster.

* **`/left_thrust_cmd`** ([std_msgs/Float32])

	Thrust command for the left wamv thruster.

#### Parameters

See configuration files in the config directory.
