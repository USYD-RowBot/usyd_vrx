#include "ros/ros.h"
#include "WaypointFollower.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wamv_waypoint_follower");
  ros::NodeHandle nh("~");

  usyd_vrx::WaypointFollower waypoint_follower(nh);

  ros::spin();
  return 0;
}