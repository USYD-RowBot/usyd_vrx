#include "ros/ros.h"
#include "CourseController.hpp"

bool hbeat_received_;

void hbeatCb(const vrx_msgs::Course::ConstPtr& msg)
{
  hbeat_received_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wamv_course_controller");
  ros::NodeHandle nh("~");

  usyd_vrx::CourseController course_controller(nh);

  float heartbeat_duration, PID_rate;
  ros::param::get("~heartbeat_duration", heartbeat_duration);
  ros::param::get("~PID_rate", PID_rate);

  // Set up heartbeat timer
  hbeat_received_ = false;
  ros::Subscriber sub_hbeat = nh.subscribe("/cmd_course", 1, hbeatCb);
  double hbeat_target_time = ros::Time::now().toSec() + heartbeat_duration;  

  ros::Rate rate(PID_rate);
  while (nh.ok())
  {
    if (ros::Time::now().toSec() > hbeat_target_time)
    {
      if (hbeat_received_ == true)
      {
        hbeat_received_ = false; 
        course_controller.enableThrusters(true); // Tell vessel to continue
      }
      else
      {
        ROS_INFO("CourseController: Thrusters disabled. Waiting for heartbeat.");     
        course_controller.enableThrusters(false); // Tell vessel to stop
      }

      hbeat_target_time = ros::Time::now().toSec() + heartbeat_duration;  
    }

    course_controller.updateController(); // Update controller regularly for consistent PID
    
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}