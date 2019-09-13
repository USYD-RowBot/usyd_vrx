/*
 * Course To Vel
 * Purpose: Implements a course (speed, yaw) controller for the WamV
 * Inputs:
 *  - Speed: Desired speed
 *  - Yaw: Desired heading
 * Outputs: Twist message
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "vrx_msgs/Course.h"

class CourseToVel
{
public:
    CourseToVel()
	{
            // ros::NodeHandle n_("~");
            // Course Publish
            velPub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

            // Get the relevant parameters
            ros::param::get("~course_kp", kp_);
            //kp_ = 0.5;

            // Start of with speed and yaw set to zero
            set_speed_ = 0.;
            set_yaw_ = 0.;

            // Don't do anything before a course command is received
            received_course_ = false;

            // Initialise the heartbeat timers
            heartbeat_timer_ = 0.;
            heartbeat_limit_ = 1.0;
            time_prev_ = ros::Time::now().toSec();


            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("/wamv/robot_localization/odometry/filtered", 1, &CourseToVel::localisationCallback, this);
            // Subscribe to course command
            courseSub_ =  n_.subscribe("/cmd_course", 1, &CourseToVel::courseCallback, this);
    }
    // Uses the odometry to get the current course, in order to find the difference
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& ekf)
	{
        // Check that a command has been received and timer is within limits
		if(ros::ok() && received_course_ && heartbeat_timer_ < heartbeat_limit_)
		{
            // Do the heartbeat checks
            double time_now = ros::Time::now().toSec();
            double time_diff;
            time_diff = time_now - time_prev_;
            heartbeat_timer_ = heartbeat_timer_ + time_diff;
            time_prev_ = time_now;

            // Convert from quaternion to RPY to get course
            tf::Quaternion quat;
            quat.setValue(ekf->pose.pose.orientation.x, ekf->pose.pose.orientation.y, ekf->pose.pose.orientation.z, ekf->pose.pose.orientation.w);
            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;

            geometry_msgs::Twist twist;
            double phid;
            // Calculate the difference between the desired and actual values
            phid = CourseToVel::constrainAngle(set_yaw_ - phi_);

            // std::cout << "Yaw: " << phi_ << "Diff: " << phid<< std::endl;
            // Fill out the twist message
            twist.linear.x = CourseToVel::limitSpeed(set_speed_, phid);
	           //twist.linear.x = 111;
            twist.linear.y = 0.;
            twist.linear.z = 0.;
            twist.angular.x = 0.;
            twist.angular.y = 0.;
            twist.angular.z = kp_*phid;

            velPub_.publish(twist);
            // std::cout << "Speed: " << twist.linear.x << " YawRate: " << twist.angular.z << std::endl;
		}
    }
    // Keeps the angle between -pi, pi
    float constrainAngle(float x)
    {
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }

    float limitSpeed(float speed, float phid){
      if (fabs(phid) > M_PI/4){
        return 0;
      }
      else{
        return speed*(1-fabs(phid/M_PI));
      }


    }
    // Receive the course message and store it
    void courseCallback(const vrx_msgs::Course::ConstPtr& msg)
    {
        if(ros::ok() && !received_odom_)
        {

            heartbeat_timer_ = 0.; //Reset the timer
            received_course_ = true; // Allow velocity commands to now be publisheds
            set_speed_ = msg->speed;
            set_yaw_ = msg->yaw;

 //geometry_msgs::Twist twist;
//	    twist.linear.x = 111;
//            twist.linear.y = 0.;
//            twist.linear.z = 0.;
//            twist.angular.x = 0.;
//            twist.angular.y = 0.;
//            twist.angular.z = 0;

//            velPub_.publish(twist);


        }
    }

private:
	ros::NodeHandle n_;
    ros::Subscriber subLocalise_;
    ros::Subscriber courseSub_;
    ros::Publisher velPub_;

	double set_speed_;

    double phi_;

    double set_yaw_;

    double kp_;

    bool received_odom_;
    bool received_course_;

    double heartbeat_timer_;
    double heartbeat_limit_;
    double time_prev_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "course_to_twist");

  CourseToVel c2v;

  ros::spin();


  return 0;
}
