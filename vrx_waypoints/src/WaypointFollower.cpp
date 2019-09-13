#include "WaypointFollower.hpp"
#include "GuidanceAlgorithms.hpp"

namespace usyd_vrx {

WaypointFollower::WaypointFollower(ros::NodeHandle& nh):
  nh_(nh)
{
  // Set up publishers.
  pub_course_      = nh_.advertise<vrx_msgs::Course>("/course_cmd", 1);
  pub_request_wps_ = nh_.advertise<std_msgs::Empty>("/request_waypoints", 1);

  WaypointFollower::setupWaypointFollower();

  // Set up waypoint command and odometry subscribers
  sub_odom_      = nh_.subscribe("/odom", 1, &WaypointFollower::odomCb, this);
  sub_waypoints_ = nh_.subscribe("/waypoints_cmd", 1, &WaypointFollower::waypointCb, this);
}

void WaypointFollower::setupWaypointFollower()
{
  ros::param::get("~nlgl_radius", nlgl_radius_);
  ros::param::get("~wp_tolerance", wp_tolerance_);
  ros::param::get("~default_speed", speed_);
  ros::param::get("~station_tolerance_pos", station_tolerance_pos_);
  ros::param::get("~station_tolerance_ang", station_tolerance_ang_);
  ros::param::get("~station_brake_distance", station_brake_distance_);

  num_wps_      =  0;
  wp_index_     = -1;
  vessel_pos_   = {0, 0};
  vessel_yaw_   =  0;
  wp_next_      = {0, 0};
  wp_prev_      = {0, 0};
}

void WaypointFollower::waypointCb(const vrx_msgs::WaypointRoute::ConstPtr& msg)
{
  ROS_INFO("WpFollower: Initialising new route.");

  waypoint_list_.clear(); // Clear all current waypoints
  wp_index_ = -1;         // Reset waypoint index

  num_wps_       = msg->waypoints.size();
  waypoint_list_ = msg->waypoints;
  speed_         = msg->speed;

  ROS_INFO_STREAM("WpFollower: New route has " << num_wps_ << " waypoints. "
    << "Running at " << speed_ << " m/s course speed.");

  station_sm_.resetStateMachine();

  wp_next_ = vessel_pos_; // Set temporary waypoint to current vessel position
  WaypointFollower::setNextWaypoint();
}

void WaypointFollower::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Convert from quaternion to RPY to get yaw
  tf::Quaternion quat; double roll, pitch, yaw;
  quat.setValue(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  vessel_yaw_ = (float) yaw; // Store current yaw

  // Store current vessel position
  vessel_pos_.x = msg->pose.pose.position.x;
  vessel_pos_.y = msg->pose.pose.position.y;

  WaypointFollower::followRoute();
}

void WaypointFollower::requestNewWaypoints()
{
  std_msgs::Empty msg;
  pub_request_wps_.publish(msg);

  ROS_INFO("WpFollower: Requesting new waypoint route.");
}

void WaypointFollower::setNextWaypoint()
{
  if (wp_index_ < num_wps_-1) // If waypoint index within bounds
  {
    wp_index_++; // Increment waypoint index

    wp_prev_.x = wp_next_.x; // Set previous waypoint
    wp_prev_.y = wp_next_.y;

    // Get next waypoint pose
    geometry_msgs::Pose pose = waypoint_list_[wp_index_].pose;

    wp_next_.x = pose.position.x; // Set next waypoint
    wp_next_.y = pose.position.y;

    // Convert from quaternion to RPY to set yaw command
    tf::Quaternion quat; double roll, pitch, yaw;
    quat.setValue(pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw_next_ = (float) yaw;

    if (wp_index_ == 0)
      ROS_INFO_STREAM("WpFollower: Now heading to waypoint "
        << WaypointFollower::getWaypointString() << ".");
    else
      ROS_INFO_STREAM("WpFollower: Waypoint reached at x=" << wp_prev_.x
        << ", y=" << wp_prev_.y << ". Now heading to waypoint "
        << WaypointFollower::getWaypointString() << ".");
  }
  else
  {
    ROS_INFO_STREAM("WpFollower: Waypoint reached at x=" << wp_next_.x
      << ", y=" << wp_next_.y << ". Route completed.");

    wp_index_++; // Increment waypoint index
    WaypointFollower::requestNewWaypoints(); // Publish waypoint request
  }
}

std::string WaypointFollower::getWaypointString()
{
  return std::to_string(wp_index_+1) + "/" + std::to_string(num_wps_);
}

void WaypointFollower::evaluateWaypoint()
{
  switch (waypoint_list_[wp_index_].nav_type) // Check waypoint type
  {
    case vrx_msgs::Waypoint::NAV_WAYPOINT:
      //
      WaypointFollower::setNextWaypoint(); // Set next waypoint
      break;

    case vrx_msgs::Waypoint::NAV_STATION:
      //
      ROS_INFO_STREAM("WpFollower: Keeping station at waypoint "
        << WaypointFollower::getWaypointString() << ".");

      double duration = (double) waypoint_list_[wp_index_].station_duration;
      station_sm_.startStation(duration); // Set up state machine and timer
      break;
  }
}

bool WaypointFollower::waypointHit(float tolerance_pos)
{
  // Distance to next waypoint
  float distance = GuidanceAlgorithms::Distance_2(vessel_pos_, wp_next_);

  return (distance < tolerance_pos); // True if hit, false otherwise
}

bool WaypointFollower::stationPoseHit(float tolerance_ang)
{
  bool position_hit = WaypointFollower::waypointHit(station_tolerance_pos_);

  float angular_error =  fabs(yaw_next_ - vessel_yaw_);

  if (angular_error > M_PI) // Smallest error around the unit circle from -PI to PI
    angular_error = 2*M_PI - angular_error;

  bool angle_hit = angular_error < tolerance_ang;

  return (angle_hit && position_hit);
}

void WaypointFollower::assignCourse(vrx_msgs::Course& msg, bool station)
{
  msg.keep_station = station;
  msg.station_yaw  = yaw_next_;

  Vec2D virtual_wp = GuidanceAlgorithms::NonlinearGuidanceLaw(
    wp_prev_, wp_next_, vessel_pos_, nlgl_radius_);
  msg.yaw = GuidanceAlgorithms::PurePursuit(virtual_wp, vessel_pos_);

  if (station) // Slow down when reaching station position
  {
    float distance_to_wp =
      GuidanceAlgorithms::Distance_2(vessel_pos_, wp_next_) - wp_tolerance_/2;

    // If less than threshold, reduce speed on approach
    if (distance_to_wp < station_brake_distance_)
      msg.speed = (distance_to_wp/station_brake_distance_)*speed_;
    else
      msg.speed = speed_;
  }
  else
    msg.speed = speed_;
}

void WaypointFollower::followRoute()
{
  vrx_msgs::Course course_cmd; // Course command for course controller

  if (num_wps_ > 0 && wp_index_ < num_wps_) // If we have route & index within bounds
  {
    switch (station_sm_.getState()) // Consult state machine for station keeping status
    {
      case StationSM::STATION_COMPLETE:
        //
        ROS_INFO("WpFollower: Station completed. Continuing on route.");
        station_sm_.resetStateMachine();
        WaypointFollower::setNextWaypoint();
        return; // Exit method, don't publish course_cmd

      case StationSM::STATION_WAIT:
        //
        WaypointFollower::assignCourse(course_cmd, true); // Align pose at station
        break;

      case StationSM::STATION_ALIGN:
        //
        if (WaypointFollower::stationPoseHit(station_tolerance_ang_))
          station_sm_.completeAlignment(); // Vessel now aligned, tell state machine

        WaypointFollower::assignCourse(course_cmd, true); // Align pose at station
        break;

      case StationSM::STATION_NONE:
        //
        if (WaypointFollower::waypointHit(wp_tolerance_)) // If next waypoint hit
          WaypointFollower::evaluateWaypoint(); // Check wp type, act accordingly

        WaypointFollower::assignCourse(course_cmd, false); // Head towards wp
        break;
    }
  }
  else
    return; // Don't publish course command

  pub_course_.publish(course_cmd); // Publish to course controller
}

}
