#include "vrx_control_2/CourseController.hpp"

namespace usyd_vrx {

CourseController::CourseController(ros::NodeHandle& nh): nh_(nh)
{
  CourseController::setupThrustController(); 

  // Set up thruster publishers.
  pub_thrust_right_ = nh_.advertise<std_msgs::Float32>("/right_thrust_cmd", 1);
  pub_thrust_left_  = nh_.advertise<std_msgs::Float32>("/left_thrust_cmd", 1);

  // Set up course command and odometry subscribers
  sub_course_ = nh_.subscribe("/cmd_course", 1, &CourseController::courseCb, this);
  sub_odom_   = nh_.subscribe("/p3d_wamv", 1, &CourseController::odomCb, this);

  tf_listener_ = new tf::TransformListener();
}

void CourseController::setupThrustController()
{
  float max_integral;
  bool use_sim_time;
  float priority_yaw_range, motor_cmd_limit, neg_scale_factor;
  float lin_Kp, lin_Ki, lin_Kd;
  float ang_Kp, ang_Ki, ang_Kd;

  // Get ROS parameters from config file.
  ros::param::get("~priority_yaw_range", priority_yaw_range);
  ros::param::get("~motor_cmd_limit", motor_cmd_limit);
  ros::param::get("~neg_scale_factor", neg_scale_factor);
  ros::param::get("~lin_Kp", lin_Kp);
  ros::param::get("~lin_Ki", lin_Ki);
  ros::param::get("~lin_Kd", lin_Kd);
  ros::param::get("~ang_Kp", ang_Kp);
  ros::param::get("~ang_Ki", ang_Ki);
  ros::param::get("~ang_Kd", ang_Kd);
  ros::param::get("~max_integral", max_integral);
  ros::param::get("~use_sim_time", use_sim_time);

  if (use_sim_time)
    ROS_INFO("Using simulator time for thruster PID control.");
  else
    ROS_INFO("Using real time for thruster PID control.");
  

  // Instantiate and configure thrust controller
  thrust_controller_ = new usyd_vrx::ThrustController(
    priority_yaw_range, motor_cmd_limit, neg_scale_factor);

  // Set up thruster PID controllers
  thrust_controller_->initLinearPID(lin_Kp, lin_Ki, lin_Kd, 
    max_integral, use_sim_time);
  thrust_controller_->initAngularPID(ang_Kp, ang_Ki, ang_Kd, 
    max_integral, use_sim_time);
}

void CourseController::courseCb(const vrx_msgs::Course::ConstPtr& msg)
{
  // Update the thrust controller with new target speed and yaw.
  thrust_controller_->setTarget(msg->speed, msg->yaw);
}

void CourseController::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Convert from quaternion to RPY to get course
  tf::Quaternion quat;
  quat.setValue(msg->pose.pose.orientation.x, 
                msg->pose.pose.orientation.y, 
                msg->pose.pose.orientation.z, 
                msg->pose.pose.orientation.w);

  // Convert quaternion to roll, pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Create vector3 with linear velocity values from odometry frame
  geometry_msgs::Vector3Stamped vel_odom, vel_base_link;
  vel_odom.vector.x = msg->twist.twist.linear.x;
  vel_odom.vector.y = msg->twist.twist.linear.y;
  vel_odom.vector.z = msg->twist.twist.linear.z;
  vel_odom.header.stamp = ros::Time();
  vel_odom.header.frame_id = "/odom";
    
  try // Transform velocity vector to base_link frame
  {
    tf_listener_->transformVector("/base_link", vel_odom, vel_base_link);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

  // Update the thrust controller with the new odometry data
  thrust_controller_->setOdometry(vel_base_link.vector.x, yaw);
}

void CourseController::updateController()
{
  double sim_time = ros::Time::now().toSec();

  float thrust_right, thrust_left;   // Get new thrust commands
  thrust_controller_->computeControlSignals(thrust_right, thrust_left, sim_time);

  std_msgs::Float32 msg_thrust_right; // Populate thrust command messages
  std_msgs::Float32 msg_thrust_left;
  msg_thrust_right.data = float(thrust_right);
  msg_thrust_left.data  = float(thrust_left);
  
  pub_thrust_right_.publish(msg_thrust_right); // Publish to the thrusters
  pub_thrust_left_.publish(msg_thrust_left);
}

}

