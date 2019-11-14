# USYD_VRX
The USYD RobotX Team's submission into the 2019 Virtual RobotX Competition

## Getting Started
This software requires you to have [ros melodic](http://wiki.ros.org/melodic) and the vrx simulation. It is reccomended to install the full-desktop version of ROS.
You will need Ubuntu Bionic or some derivative.

To Get started please have an up to date version of the vrx simulation installed. Please install it from source using the instructions [here](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).


### Prequisites

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
Two other prerequisites, `hector-gazebo-plugins` and `pointcloud-to-laserscan` also need to be installed. A shell script in vrx_deploy can be used to do this.
```
./vrx_deploy/installEverything.sh
```



And you should be ready to go!


## Launching

### Barebones
To launch the base system without sensors or course elements:
```
roslaunch vrx_bringup barebones.launch
```
To launch with sensors and obstacles:
```
roslaunch vrx_bringup sensored_boat.launch
```
To launch a competition:
```
roslaunch vrx_tasks runtask.launch task:=<task> gui:=<true | false> 
```

Where task may be one of:

- dock
- navigation_task
- perception_task
- sandisland
- scan_and_dock
- station_keeping
- wayfinding

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

```roslaunch vrx_control course_controller.launch```

With these two nodes you can publish a message to /cmd_course of message type vrx_msgs/Course, and it will try to follow direction. e.g :
```rostopic pub -r 10 /cmd_course vrx_msgs/Course  '{speed: 1.0, yaw: 0.0}'```

### Waypoint Following

```roslaunch vrx_bringup sensored_boat.launch```

```roslaunch vrx_control control.launch```

This launches the Course Controller and Waypoint Follower, the latter of which listens for a vrx_msgs/WaypointRoute message on the topic /waypoints_cmd. When it receives a WaypointRoute message, it will follow the path until complete.

### Docking

To test the docking program, run the following in separate terminals:

```roslaunch vrx_bringup sensored_boat.launch override_location:=true```

```roslaunch vrx_control control.launch```

```roslaunch vrx_navigation docking.launch```

The WAMV will spawn in front of the dock. To attempt a docking procedure: 

```rosrun vrx_navigation tmp_docking_client.py```

This will generate an actionlib client to send a goal to the docking program. The WAMV should enter the dock, hold, then return to where it was when you started the docking client. To view feedback on the progress of the docking procedure, you can use the following:

```rostopic echo /wamv/docking/feedback```

### Headless mode
Wherever you see

```roslaunch vrx_bringup sensored_boat.launch```

, you can replace it with

```roslaunch vrx_bringup sensored_boat.launch gui:=false```

 and it will run the simulation with only RVIZ as a gui, running no Gazebo gui. This should reduce the CPU load of running the simulation.

#### Path Planner
To use a demonstrate a path planner with the waypoint_follower node, run these in seperate tabs along with the previous nodes (without simple waypoints).

Launching Rviz
```
roslaunch vrx_bringup rviz.launch
```
Launch the Lidar Node

```
roslaunch vrx_navigation lidar_node.launch
```

Now you should be able to set a 2d goal pose through rviz and the wam_v will attempt to follow that path.
