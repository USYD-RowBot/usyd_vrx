#pragma once

#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "vrx_control_2/ThrustController.hpp"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "vrx_msgs/Course.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"

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
    * Calculates next thruster PID controller outputs. To be run 
    * at a consistent rate.
    */
    void updateController();

  private:
    //! ROS node handle.
    ros::NodeHandle nh_;

    //! ROS subscriber for course commands.
    ros::Subscriber sub_course_;

    //! ROS subscriber for odometry data.
    ros::Subscriber sub_odom_;

    //! ROS publisher for right thruster.
    ros::Publisher  pub_thrust_right_;

    //! ROS publisher for left thruster.
    ros::Publisher  pub_thrust_left_;

    //! ROS transform listener.
    tf::TransformListener* tf_listener_;

    //! Pointer to thrust controller to provide commands to thrusters.
    usyd_vrx::ThrustController* thrust_controller_;

    /*!
    * Instantiate and configure thrust controller.
    */
    void setupThrustController();

    /*!
    * Callback method for course commands.
    * @param msg the received message.
    */
    void courseCb(const vrx_msgs::Course::ConstPtr& msg);

    /*!
    * Callback method for odometry data.
    * @param msg the received message.
    */
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
};

}