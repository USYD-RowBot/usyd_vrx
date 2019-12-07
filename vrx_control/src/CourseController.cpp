#include "CourseController.hpp"

namespace usyd_vrx {

CourseController::CourseController(ros::NodeHandle& nh): nh_(nh)
{
  CourseController::setupThrustController();

  thrusters_enabled_ = false;

  // Set up thruster publishers.
  pub_thrust_right_       = nh_.advertise<std_msgs::Float32>("/right_thrust_cmd", 1);
  pub_thrust_right_angle_ = nh_.advertise<std_msgs::Float32>("/right_thrust_angle", 1, true);
  pub_thrust_left_        = nh_.advertise<std_msgs::Float32>("/left_thrust_cmd", 1);
  pub_thrust_left_angle_  = nh_.advertise<std_msgs::Float32>("/left_thrust_angle", 1, true);
  pub_thrust_lat_         = nh_.advertise<std_msgs::Float32>("/lateral_thrust_cmd", 1);

  // Set up course command and odometry subscribers
  sub_course_ = nh_.subscribe("/course_cmd", 1, &CourseController::courseCb, this);
  sub_odom_   = nh_.subscribe("/odom", 1, &CourseController::odomCb, this);

  tf_listener_ = new tf::TransformListener();

  // Make sure thrusters are reset to zero angles on startup: TODO doesn't work :/
  CourseController::reconfigureThrusters(ThrustSM::THRUST_RECONFIG_STRAIGHT);
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
  float reconfig_duration, station_tolerance_ang;
  float priority_yaw_range, motor_cmd_limit;
  float lateral_scale_x, lateral_scale_y, neg_scale_factor;
  float lin_Kp, lin_Ki, lin_Kd, max_integral;
  float ang_Kp, ang_Ki, ang_Kd;
  float stn_Kp, stn_Ki, stn_Kd;

  // Get ROS parameters from config file.
  ros::param::get("~thrust_config", thrust_config);
  ros::param::get("~priority_yaw_range", priority_yaw_range);
  ros::param::get("~motor_cmd_limit", motor_cmd_limit);
  ros::param::get("~lateral_scale_x", lateral_scale_x);
  ros::param::get("~lateral_scale_y", lateral_scale_y);
  ros::param::get("~neg_scale_factor", neg_scale_factor);
  ros::param::get("~lin_Kp", lin_Kp);
  ros::param::get("~lin_Ki", lin_Ki);
  ros::param::get("~lin_Kd", lin_Kd);
  ros::param::get("~ang_Kp", ang_Kp);
  ros::param::get("~ang_Ki", ang_Ki);
  ros::param::get("~ang_Kd", ang_Kd);
  ros::param::get("~stn_Kp", stn_Kp);
  ros::param::get("~stn_Ki", stn_Ki);
  ros::param::get("~stn_Kd", stn_Kd);
  ros::param::get("~max_integral", max_integral);
  ros::param::get("~use_sim_time", use_sim_time);
  ros::param::get("~reconfig_duration", reconfig_duration);
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
    priority_yaw_range, motor_cmd_limit, lateral_scale_x, lateral_scale_y,
    neg_scale_factor, station_tolerance_ang);

  // Set up thruster PID controllers
  thrust_controller_->initLinearPID(lin_Kp, lin_Ki, lin_Kd,
    max_integral, use_sim_time);
  thrust_controller_->initAngularPID(ang_Kp, ang_Ki, ang_Kd,
    max_integral, use_sim_time);
  thrust_controller_->initStationPID(stn_Kp, stn_Ki, stn_Kd,
    max_integral, use_sim_time);

  // Instantiate thrust state machine
  thrust_sm_ = new usyd_vrx::ThrustSM(reconfig_duration);
}

void CourseController::reconfigureThrusters(ThrustSM::THRUST_STATE state)
{
  std_msgs::Float32 msg;

  if (state == ThrustSM::THRUST_RECONFIG_STRAIGHT)
    msg.data = 0;

  else if (state == ThrustSM::THRUST_RECONFIG_LATERAL)
    msg.data = M_PI_4; // 45 degrees

  else
    return; // Don't publish if not in correct state

  pub_thrust_right_angle_.publish(msg); // Publish to thrusters

  msg.data = -msg.data; // Left thruster angle opposes right thruster
  pub_thrust_left_angle_.publish(msg); 
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

    case ThrustSM::THRUST_STATION:
      // Tell thrust controller to work out thrust proportions for strafing.
      thrust_controller_->setStrafeProportions(msg->yaw);

      // Update the thrust controller with strafe thrust and station yaw.
      thrust_controller_->setTarget(0, msg->station_yaw);

      // Update the thrust controller with distance to station.
      thrust_controller_->setStationTarget(msg->station_dist_x, msg->station_dist_y);
      break;
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

  //std::cout << "reported x velocity is " << msg->twist.twist.linear.x << std::endl;

  // Update the thrust controller with the new odometry data
  thrust_controller_->setOdometry(msg->twist.twist.linear.x, yaw);
}

void CourseController::enableThrusters(bool enabled)
{
  // Reset PIDs if reactivating vessel thrusters
  if (thrusters_enabled_ == false && enabled == true)
    thrust_controller_->resetPIDs();

  thrusters_enabled_ = enabled; // Set thrusters enabled or disabled

  if (enabled == false) // If disabling thrusters, return SM to traverse
    thrust_sm_->updateState(false);
}

void CourseController::updateController()
{
  float thrust_right, thrust_left, thrust_lateral; // Thrust values
  double sim_time = ros::Time::now().toSec();

  switch (thrust_sm_->getState()) // Get current state machine state
  {
    case ThrustSM::THRUST_TRAVERSE:
      thrust_controller_->getControlSignalTraverse(
        thrust_right, thrust_left, thrust_lateral, sim_time); // Normal traverse
      break;

    case ThrustSM::THRUST_RECONFIG_STRAIGHT:
      thrust_controller_->resetPIDs(); return; // Reset PIDs when reconfiguring

    case ThrustSM::THRUST_RECONFIG_LATERAL:
      thrust_controller_->resetPIDs(); return; // Reset PIDs when reconfiguring

    case ThrustSM::THRUST_STATION:
      thrust_controller_->getControlSignalStation( // Station keeping control
        thrust_right, thrust_left, thrust_lateral, sim_time);
      break;
  }

  if (thrusters_enabled_ == true)
  {
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

}
