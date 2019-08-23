# USYD_VRX
The USYD RobotX Team's submission into the 2019 Virtual RobotX Competition

## Getting Started
This software requires you to have [ros melodic](http://wiki.ros.org/melodic) and the vrx simulation. It is reccomended to install the full-desktop version of ROS.
You will need Ubuntu Bionic or some derivative.

To Get started please have an up to date version of the vrx simulation installed. Please install it from source using the instructions [here](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).


### Prequisites

You need to install pointcloud_to_laserscan:
```
sudo apt-get install ros-melodic-pointcloud-to-laserscan
```


### Installing
To install run these commands
```
cd ~/vrx_ws/src
git clone https://github.com/USYD-RowBot/usyd_vrx.git
cd ..
catkin_make

echo "source ~/vrx_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
And you should be ready to go!

## Launching

### Barebones
To launch the base system without sensors or course elements:
```
roslaunch roslaunch vrx_bringup barebones.launch
```
To launch with sensors and obstacles:
```
roslaunch roslaunch vrx_bringup sensored_boat.launch
```

### Using mapping
To launch with mapping there are two ways.
1. Launch 2 nodes (reccomended)
```
roslaunch vrx_bringup sensored_boat.launch
```
And in a seperate tab:
```
roslaunch vrx_navigation lidar_node.launch
```

2. Launch all in one file.
```
roslaunch vrx_bringup mapping_bringup.launch
```

### Basic Control

Prequisites:
```
sudo apt-get install ros-melodic-global-planner
```



Launch in separate tabs:
```roslaunch vrx_bringup sensored_boat.launch```

```roslaunch vrx_control_2 course_controller.launch```

With these two nodes you can publish a message to /cmd_course  of message type vrx_msgs/Course, and it will try to follow direction. e.g :
```rostopic pub -r 10 /cmd_course vrx_msgs/Course  '{speed: 1.0, yaw: 0.0}'```

### Waypoint Following

```roslaunch vrx_bringup sensored_boat.launch```

```roslaunch vrx_control waypoint_following.launch```

This Controller listens for a nav_msgs/Path message on the topic /waypoints, when it receives a path message it will follow the path until it is completed.

To test with an example path:

```roslaunch simple_waypoints simple_waypoints.launch```

### Headless mode
Wherever you see 

```roslaunch vrx_bringup sensored_boat.launch```

, you can replace it with 

```roslaunch vrx_bringup sensored_boat_nogui.launch```

 and it will run the simulation with RVIZ as a gui instead of Gazebo. That should reduce the CPU load of running the simulation.

#### Path Planner
To use a demonstrate a path planner with the waypoint_following node, run these in seperate tabs along with the previous nodes (without simple waypoints).

Launching Rviz
```
roslaunch vrx_bringup rviz.launch
```
Launch the Lidar Node

```
roslaunch vrx_navigation lidar_node.launch
```

Now you should be able to set a 2d goal pose through rviz and the wam_v will attempt to follow that path.
