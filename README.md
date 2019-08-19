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
roslaunch vrx_bringup barebones.launch
```
To launch with sensors and obstacles:
```
roslaunch vrx_bringup sensored_boat.launch
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

