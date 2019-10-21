#include "ros/ros.h"
#include "WaypointFollower.hpp"
#include "std_srvs/SetBool.h"

usyd_vrx::WaypointFollower* waypoint_follower;

bool enable_follower_srv(std_srvs::SetBool::Request  &req,
                         std_srvs::SetBool::Response &res)
{
  waypoint_follower->enable(req.data);  
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wamv_waypoint_follower");
  ros::NodeHandle nh("~");
  
  waypoint_follower = new usyd_vrx::WaypointFollower(nh);
  ros::ServiceServer _srv = nh.advertiseService("enable_follower", enable_follower_srv);

  ros::spin();

  delete waypoint_follower;
  return 0;
}