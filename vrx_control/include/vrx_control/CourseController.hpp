#pragma once

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "vrx_msgs/Course.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "ThrustController.hpp"
#include "ThrustSM.hpp"

namespace usyd_vrx {

class CourseController
{
  public:
    /*!
    * Constructor.
    * @param nh the ROS node handle.
    */
    CourseController(ros::NodeHandle& nh);

    /*!
    * Destructor.
    */
    ~CourseController();

    /*!
    * Calculates next thruster PID controller outputs. To be run 
    * at a consistent rate.
    */
    void updateController();

    /*!
    * Enables or disables thrust command publishers. Resets thrust state machine.
    */
    void enableThrusters(bool enabled);

  private:
    //! ROS node handle.
    ros::NodeHandle nh_;

    //! ROS subscriber for course commands.
    ros::Subscriber sub_course_;

    //! ROS subscriber for odometry data.
    ros::Subscriber sub_odom_;

    //! ROS publisher for right thruster.
    ros::Publisher  pub_thrust_right_;

    //! ROS publisher for right thruster angle.
    ros::Publisher  pub_thrust_right_angle_;

    //! ROS publisher for left thruster.
    ros::Publisher  pub_thrust_left_;

    //! ROS publisher for left thruster angle.
    ros::Publisher  pub_thrust_left_angle_;

    //! ROS publisher for lateral thruster.
    ros::Publisher  pub_thrust_lat_;

    //! ROS transform listener.
    tf::TransformListener* tf_listener_;

    //! Pointer to thrust controller to provide commands to thrusters.
    usyd_vrx::ThrustController* thrust_controller_;

    //! State machine for thrust operations.
    usyd_vrx::ThrustSM* thrust_sm_;

    //! Controls whether thruster msgs are published or not.
    bool thrusters_enabled_;

    //! Letter indicating thrust configuration. "H" differential, "T" lateral.
    char thrust_config_;

    /*!
    * Instantiate and configure thrust controller.
    */
    void setupThrustController();

    /*!
    * Publish message to thrusters to change angle.
    * @state current state of state machine, either THRUST_RECONFIG_STRAIGHT or
    *   THRUST_RECONFIG_LATERAL are viable.
    */
    void reconfigureThrusters(ThrustSM::THRUST_STATE state);

    /*!
    * Callback method for odometry data.
    * @param msg the received message.
    */
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

    /*!
    * Callback method for course commands.
    * @param msg the received message.
    */
    void courseCb(const vrx_msgs::Course::ConstPtr& msg);
};

}