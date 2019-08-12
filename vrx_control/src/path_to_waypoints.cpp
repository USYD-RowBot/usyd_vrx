/*
 * Path To Waypoints
 * Purpose: Receives a nav_msgs/Path and makes it a nav_msgs/VesselPath
 * Inputs:
 *  - Path: nav_msgs/Path
 * Outputs:
 *  - Waypoints: vrx_msgs/VesselPath
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"
#include "vrx_msgs/VesselWaypoint.h"
#include "vrx_msgs/VesselPath.h"

class PathToWaypoints
{
public:
    PathToWaypoints()
	{
            // ros::NodeHandle n_("~");
            waypointPub_ = n_.advertise<vrx_msgs::VesselPath>("waypoints", 1);
            pathSub_ =  n_.subscribe("path", 1, &PathToWaypoints::pathCallback, this);
            ros::param::get("~tolerance", tolerance_);
    }

    //Path Callback
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        vrx_msgs::VesselPath wpPath;
        for(int n=0;n<msg->poses.size(); n++)
        {
            vrx_msgs::VesselWaypoint wp;
            wp.pose = msg->poses.at(n).pose;
            wp.tolerance = tolerance_;
            wpPath.waypoints.push_back(wp);
        }
        waypointPub_.publish(wpPath);

    }

private:
	ros::NodeHandle n_;
    ros::Subscriber pathSub_;
    ros::Publisher waypointPub_;

    double tolerance_;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_to_waypoint");

  PathToWaypoints p2w;

  ros::spin();


  return 0;
}
