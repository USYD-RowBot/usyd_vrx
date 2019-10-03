#pragma once

#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "vrx_msgs/Course.h"
#include "vrx_msgs/Waypoint.h"
#include "vrx_msgs/WaypointRoute.h"
#include "geometry_msgs/Pose.h"
#include "StationSM.hpp"
#include "GuidanceAlgorithms.hpp"

namespace usyd_vrx {

typedef usyd_vrx::GuidanceAlgorithms::Vec2D Vec2D; 

class WaypointFollower
{
  public:
    /*!
    * Constructor.
    * @param nh the ROS node handle.
    */
    WaypointFollower(ros::NodeHandle& nh);

    /*!
    * Enable or disable the waypoint follower.
    * @param do_enable enable when true, disable when false.
    */
    void enable(bool do_enable);

  private:
    //! ROS node handle.
    ros::NodeHandle nh_;

    //! ROS publisher for giving course commands to vessel.
    ros::Publisher pub_course_;

    //! ROS publisher for requesting new waypoints.
    ros::Publisher pub_request_wps_;

    //! ROS subscriber for vessel odometry data.
    ros::Subscriber sub_odom_;

    //! ROS subscriber for reading in new waypoints.
    ros::Subscriber sub_waypoints_;
    
    //! Circle radius for nonlinear guidance law calculation.
    float nlgl_radius_;

    //! Tolerance for detecting a waypoint hit.
    float wp_tolerance_;

    //! Distance from station position to start reducing speed.
    float station_brake_distance_;

    //! Tolerance for detecting station position alignment.
    float station_tolerance_pos_;

    //! Tolerance for detecting station angular alignment.
    float station_tolerance_ang_;

    //! Default thrust to strafe with when 1 meter from station wp.
    float station_default_thrust_;

    //! Speed to use for course commands.
    float speed_;

    //! Vector of waypoints in requested route.
    std::vector<vrx_msgs::Waypoint> waypoint_list_;

    //! Number of waypoints on route.
    int num_wps_; 

    //! Current target index on waypoint route.
    int wp_index_;

    usyd_vrx::StationSM station_sm_;

    //! Current vessel position.
    Vec2D vessel_pos_;

    //! Current vessel yaw.
    float vessel_yaw_;

    //! Next waypoint position on route.
    Vec2D wp_next_;

    //! Desired yaw at next waypoint position, for station keeping.
    float yaw_next_;

    //! Previous waypoint position on route.
    Vec2D wp_prev_;
    
    /*!
    * Initialises private member variables.
    */
    void setupWaypointFollower();

    /*!
    * Looks at the waypoint type of the current waypoint and applies 
    *   the appropriate action. 
    */
    void evaluateWaypoint();

    /*!
    * Detects whether or not we have reached the target waypoint.
    * @param tolerance_pos tolerance for detecting positional match.
    * @return true if waypoint hit, false otherwise.
    */
    bool waypointHit(float tolerance_pos);

    /*!
    * Detects whether or not we have aligned the target pose.
    * @param tolerance_pos tolerance for detecting angular match.
    * @return true if pose aligned, false otherwise.
    */
    bool stationPoseHit(float tolerance_ang);

    /*!
    * Calculates & populates Course msg fields, yawing towards target with NLGL.
    * @param msg course message to be populated.
    * @param station whether or not we are station keeeping.
    */
    void assignCourse(vrx_msgs::Course& msg, bool station);

    /*!
    * Set next target waypoint position from waypoint route list, set target yaw.
    * @return boolean, true if able to set waypoint, false if end of route.
    */
    void setNextWaypoint();

    /*!
    * Publishes request for new waypoint route.
    */
    void requestNewWaypoints();

    /*!
    * Gets a formatted string representing which waypoint we are at.
    * @return string current progression through route, e.g. "2/4".
    */
    std::string getWaypointString();

    /*!
    * Follow the given route and send course commands to the course controller.
    */
    void followRoute();

    /*!
    * Callback method for odometry data.
    * @param msg the received message.
    */
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

    /*!
    * Callback method for reading new waypoint route.
    * @param msg the received message.
    */
    void waypointCb(const vrx_msgs::WaypointRoute::ConstPtr& msg);
};

}