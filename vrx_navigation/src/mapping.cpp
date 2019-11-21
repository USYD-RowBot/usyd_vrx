/* Created by John Sumskas and Lachlan Townshend for MTRX3760*/
//TODO Check if scan has been recieved.
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <vrx_gazebo/Task.h>
#include <string>

class MappingServer{
    /*Mapping Server to Create a map from odometry well*/
private:
  sensor_msgs::LaserScan scan;
  nav_msgs::OccupancyGrid map;
  ros::NodeHandle node;
  geometry_msgs::TransformStamped odom_transform;
  int width ;
  float resolution;
  float offset_x;
  float offset_y;
  float range_min_;

public:
  MappingServer(ros::NodeHandle node){
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    this->node = node;

    ros::param::get("~width", width);
    ros::param::get("~resolution", resolution);
    ros::param::get("~offset_x", offset_x);
    ros::param::get("~offset_y", offset_y);

    vrx_gazebo::Task task_msg = *(ros::topic::waitForMessage<vrx_gazebo::Task>("/vrx/task/info"));
    std::string task_name = task_msg.name;

    if (task_name == "perception")
      range_min_ = 2.0;
    else
      range_min_ = 8.0;

    ROS_DEBUG("range_min_ is %f metres.", range_min_ );

    // width = 1024;
    // resolution = 0.4;
    // offset_x = 60;
    // offset_y= 180;

    // Initalize an empty Map
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = width;
    map.info.origin.position.x = -float(width)*resolution/2 + offset_x;
    map.info.origin.position.y = -float(width)*resolution/2 + offset_y;
    map.info.origin.orientation.w = 1.0;
    map.header.frame_id = "map";
    map.data.clear();
    for(long int i = 0; i< width*width; i++){
      map.data.push_back(-1);
    }

    //Wait for the first odom transformation to be heard.
    ROS_INFO("Waiting for odometry.");
    while(ros::ok()){
      try{

        //TODO get tf_prefix from params
        odom_transform = tf_buffer.lookupTransform("odom","wamv/lidar_wamv_link",ros::Time(0));
        ROS_INFO("Found odometry.");
        break;
      }
      catch (tf2::TransformException &ex){
        ROS_DEBUG("ODOM Transform Not found");
      }
    }
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg){
      // Scan callback to hold scan in history.
    ROS_DEBUG("RECIEVED SCAN");
    scan = *scan_msg;
  }

  void publishMap(ros::Publisher* map_publisher){
      //Publish the map on the channel.
    ROS_DEBUG("Publishing map");
    map_publisher->publish(map);
  //  ROS_INFO("PUBLISHING ODOM--> MAP");
  //  static tf::TransformBroadcaster br;
  //  tf::Transform transform;
    //tf::Quaternion q;
    //q.setRPY(0, 0, 0);
    //transform.setRotation(q);
  //  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
  }

  void getOdom(tf2_ros::Buffer *tf_buffer){
      //Attempt to get the odometry transform and save it.
    try{
      odom_transform = tf_buffer->lookupTransform("odom","wamv/lidar_wamv_link",ros::Time(0));
      ROS_DEBUG("Found odom_transform.");
    }
    catch (tf2::TransformException &ex){
      ROS_DEBUG("ODOM Transform Not found");
    }

  }

  int clampMax(int num, int max)
  {
    if (num > max)
      return max;
    else if (num < 0)
      return 0;
    else
      return num;
  }

  void generateMap(){
      //Given laserscans and map, attempt to generate the map.
      //TODO Check if first scan is has been recieved.
    ROS_DEBUG("Generating map");

    //Calculate angle relative to the map.
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom_transform.transform.rotation,q);
    tf::Matrix3x3 matrix(q);
    double roll, pitch, my_yaw;
    matrix.getRPY(roll,pitch,my_yaw);

    float angle = my_yaw + scan.angle_min; //First scan Resultant angle relative to the map.
    ROS_DEBUG("First scan angle is at %f", angle);

    //Get laserscan info
    float angle_max = scan.angle_max;
    float angle_min = scan.angle_min;
    float angle_increment = scan.angle_increment;
    float range_max = scan.range_max;
    //float range_min = scan.range_min;
    float range_min = range_min_;

    //Amount of points that need to be calculated.
    int c = int((angle_max - angle_min)/angle_increment);

    int index_max = map.info.width*map.info.width - 1;

    ROS_DEBUG("Running %d scan points", c);
    //Iterate through each point.
    for (int i = 0; i < c; i++){
      float range = scan.ranges[i];
      //If it is a valud reading.
      //set pixel on map;
      //TODO Change originxd and y, to be defined of map origin. Right now assume its off the centre.
      //TODO Change hard values to paramer values from map info or from rosparam

      int origin_x = width/2 - int(offset_x/resolution); //Starting location
      int origin_y = width/2 - int(offset_y/resolution);
      int increment_value = 8; //Value to increment occupied space
      int decrement_value = 2; //Value to decrement occupied space.

      //Determine current location on map from odometry.
      int odom_x = origin_x+(odom_transform.transform.translation.x)/resolution;
      int odom_y = origin_y+(odom_transform.transform.translation.y)/resolution;

      //Determing detected occpued space from scan relative to current position.
      int x = int(odom_x + (range/resolution)*cos(angle));
      int y = int(odom_y + (range/resolution)*sin(angle));
      int index  = clampMax(y*width + x, index_max);
      int index2 = clampMax(odom_y*width + odom_x, index_max);

      if (range < range_max)
      {
        ROS_DEBUG("RANGE: %f angle: %f", range,angle);


        map.data[index2] = 0;//Set odom to free space (Used for debugging location tracking on map)
        //If Occupied increment untill 100.
        if (map.data[index] < (100-increment_value)){
          map.data[index] = map.data[index]+increment_value;
        }
        else {
          map.data[index] = 100;
        }

        //Iterate through range less range, and set those pixels to free pixels untill a lower limit of 0.
        for (int j = int(range/resolution); j>int(range_min/resolution);j--){
          int x = int(odom_x + float(j)*cos(angle));
          int y = int(odom_y + float(j)*sin(angle));
          int index = clampMax(y*width + x, index_max);
          if (map.data[index] > decrement_value)
          {
            if ((float(j)*resolution) <= 12 && (float(j)*resolution) >= 5){
              map.data[index] = map.data[index]-decrement_value;
            }
          }
          else {
            map.data[index] = 0;
          }
        }
      }
      else{
        float clear_range = 30;

        for (int j = int(clear_range/resolution); j>int(range_min/resolution);j--){
          int x = int(odom_x + float(j)*cos(angle));
          int y = int(odom_y + float(j)*sin(angle));
          int index = clampMax(y*width + x, index_max);
          if (map.data[index] > decrement_value)
          {
            //If distance is < clear range and > 5
            if ((float(j)*resolution) <= clear_range && (float(j)*resolution) >= 5){
              map.data[index] = map.data[index]-decrement_value;
            }
          }
          else {
            map.data[index] = 0;
          }
        }
      }
      angle = angle + angle_increment; //Increment angle
    }
  }
  //TODO Reset map function.
  //TODO FOR assignment, reset map of recieveing tf_static.
};

int main(int argc, char **argv){
  ros::init(argc,argv, "mapping");
  ros::NodeHandle n;
  MappingServer mapping_server(n);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Subscriber scan_sub = n.subscribe("scan",10,&MappingServer::scanCallback, &mapping_server);
  ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("map",10);

  //TODO Recieve rate from rosparam. GET VELODYNE LOCATION FROM ROS
  int rate_hz = 10;
  ros::Rate rate(rate_hz);
  while(ros::ok()){
    mapping_server.getOdom(&tf_buffer);
    mapping_server.generateMap();
    mapping_server.publishMap(&map_publisher);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
