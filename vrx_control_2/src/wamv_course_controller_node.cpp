#include "ros/ros.h"
#include "vrx_control_2/CourseController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wamv_course_controller");
  ros::NodeHandle nh("~");

  usyd_vrx::CourseController course_controller(nh);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    course_controller.updateController(); // Update controller regularly for consistent PID
    
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}