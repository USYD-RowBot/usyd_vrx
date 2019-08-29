#include "CourseController.hpp"

namespace usyd_vrx {

CourseController::CourseController(ros::NodeHandle& nh): nh_(nh)
{
  CourseController::setupThrustController(); 

  // Set up thruster publishers.
  pub_thrust_right_       = nh_.advertise<std_msgs::Float32>("/right_thrust_cmd", 1);
  pub_thrust_right_angle_ = nh_.advertise<std_msgs::Float32>("/right_thrust_angle", 1);
  pub_thrust_left_        = nh_.advertise<std_msgs::Float32>("/left_thrust_cmd", 1);
  pub_thrust_lat_ = nh_.advertise<std_msgs::Float32>("/lateral_thrust_cmd", 1);

  // Set up course command and odometry subscribers
  sub_course_ = nh_.subscribe("/cmd_course", 1, &CourseController::courseCb, this);
  sub_odom_   = nh_.subscribe("/p3d_wamv", 1, &CourseController::odomCb, this);

  tf_listener_ = new tf::TransformListener();
}

CourseController::~CourseController()
{
  delete tf_listener_; // Delete objects created with "new" keyword
  delete thrust_controller_;
  delete thrust_sm_;
}

void CourseController::setupThrustController()
{
  bool use_sim_time; 
  std::string thrust_config;
  float reconfig_duration, strafe_duration, strafe_thrust, station_tolerance_ang;
  float priority_yaw_range, motor_cmd_limit;
  float lateral_scale_factor, neg_scale_factor;
  float lin_Kp, lin_Ki, lin_Kd, max_integral;
  float ang_Kp, ang_Ki, ang_Kd;

  // Get ROS parameters from config file.
  ros::param::get("~thrust_config", thrust_config);
  ros::param::get("~priority_yaw_range", priority_yaw_range);
  ros::param::get("~motor_cmd_limit", motor_cmd_limit);
  ros::param::get("~lateral_scale_factor", lateral_scale_factor);
  ros::param::get("~neg_scale_factor", neg_scale_factor);
  ros::param::get("~lin_Kp", lin_Kp);
  ros::param::get("~lin_Ki", lin_Ki);
  ros::param::get("~lin_Kd", lin_Kd);
  ros::param::get("~ang_Kp", ang_Kp);
  ros::param::get("~ang_Ki", ang_Ki);
  ros::param::get("~ang_Kd", ang_Kd);
  ros::param::get("~max_integral", max_integral);
  ros::param::get("~use_sim_time", use_sim_time);
  ros::param::get("~strafe_duration", strafe_duration);
  ros::param::get("~reconfig_duration", reconfig_duration);
  ros::param::get("~strafe_thrust", strafe_thrust);
  ros::param::get("~station_tolerance_ang", station_tolerance_ang);

  thrust_config_ = thrust_config[0]; // Convert to char
  ROS_INFO_STREAM("CourseController: Vessel is using " << thrust_config_ 
    << " thruster configuration.");

  if (use_sim_time)
    ROS_INFO("CourseController: Using simulator time for thruster PID control.");
  else
    ROS_INFO("CourseController: Using real time for thruster PID control.");
  
  // Instantiate and configure thrust controller
  thrust_controller_ = new usyd_vrx::ThrustController(thrust_config_,
    priority_yaw_range, motor_cmd_limit, lateral_scale_factor, 
    neg_scale_factor, strafe_thrust, station_tolerance_ang);

  // Set up thruster PID controllers
  thrust_controller_->initLinearPID(lin_Kp, lin_Ki, lin_Kd, 
    max_integral, use_sim_time);
  thrust_controller_->initAngularPID(ang_Kp, ang_Ki, ang_Kd, 
    max_integral, use_sim_time);

  // Instantiate thrust state machine
  thrust_sm_ = new usyd_vrx::ThrustSM(strafe_duration, reconfig_duration);
}

void CourseController::reconfigureThrusters(ThrustSM::THRUST_STATE state)
{
  std_msgs::Float32 msg;

  if (state == ThrustSM::THRUST_RECONFIG_STRAIGHT)
    msg.data = 0;      // TODO work out if 0 rad is straight

  else if (state == ThrustSM::THRUST_RECONFIG_LATERAL)
    msg.data = M_PI_2; // TODO work out if pi/2 rad is lateral

  else
    return; // Don't publish if not in correct state
  
  pub_thrust_right_angle_.publish(msg); // Publish to thruster
}

void CourseController::courseCb(const vrx_msgs::Course::ConstPtr& msg)
{
  // Only permit station keeping in T thruster configuration.
  if (msg->keep_station && thrust_config_ != 'T') 
  {
    ROS_INFO_STREAM("CourseController: Can only perform station keeping "
      << "operation in T thruster configuration.");
    thrust_controller_->setTarget(0.0, 0.0); // Stop boat
    return;
  }   

  thrust_sm_->updateState(msg->keep_station); // Update state machine

  ROS_INFO_STREAM("ThrustSM state is " << thrust_sm_->getState());

  switch (thrust_sm_->getState()) // Get current state machine state
  {
    case ThrustSM::THRUST_TRAVERSE:
      // Update the thrust controller with new target speed and yaw.
      thrust_controller_->setTarget(msg->speed, msg->yaw);
      break;

    case ThrustSM::THRUST_RECONFIG_STRAIGHT:
      CourseController::reconfigureThrusters(ThrustSM::THRUST_RECONFIG_STRAIGHT);
      break;

    case ThrustSM::THRUST_RECONFIG_LATERAL:
      CourseController::reconfigureThrusters(ThrustSM::THRUST_RECONFIG_LATERAL);
      break;

    case ThrustSM::THRUST_ROTATE:
      // Update the thrust controller with zero speed and station yaw.
      thrust_controller_->setTarget(0.0, msg->station_yaw);
      break;

    case ThrustSM::THRUST_STRAFE:
      // Tell thrust controller to work out thrust proportions for strafing.
      thrust_controller_->setStrafeProportions(msg->yaw);
  }
}

void CourseController::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Convert from quaternion to RPY to get yaw
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
  float thrust_right, thrust_left, thrust_lateral; // Thrust values
  double sim_time = ros::Time::now().toSec();

  switch (thrust_sm_->getState()) // Get current state machine state
  {
    case ThrustSM::THRUST_TRAVERSE:    
      thrust_controller_->getControlSignalTraverse(
        thrust_right, thrust_left, sim_time); // Normal traverse
      break;

    case ThrustSM::THRUST_RECONFIG_STRAIGHT:
      thrust_controller_->resetPIDs(); return; // Reset PIDs when reconfiguring

    case ThrustSM::THRUST_RECONFIG_LATERAL:
      thrust_controller_->resetPIDs(); return; // Reset PIDs when reconfiguring

    case ThrustSM::THRUST_ROTATE:
      if (thrust_controller_->stationAngleHit()) // Check if station angle aligned
      {
        thrust_sm_->reportRotationComplete(); // Tell state machine 
        return;
      }
      else
        thrust_controller_->getControlSignalRotate( // Rotate for station keeping
          thrust_right, thrust_lateral, sim_time);
      break; 
      
    case ThrustSM::THRUST_STRAFE: 
      thrust_controller_->getStrafingThrust(      // Station keeping strafing
        thrust_right, thrust_left, thrust_lateral);
      break;
  }

  std_msgs::Float32 msg_thrust_right; // Populate messages
  std_msgs::Float32 msg_thrust_left;
  std_msgs::Float32 msg_thrust_lateral;
  msg_thrust_right.data   = float(thrust_right);
  msg_thrust_left.data    = float(thrust_left);
  msg_thrust_lateral.data = float(thrust_lateral);

  pub_thrust_right_.publish(msg_thrust_right); // Publish to thrusters
  pub_thrust_left_.publish(msg_thrust_left);
  pub_thrust_lat_.publish(msg_thrust_lateral);  
}

}

