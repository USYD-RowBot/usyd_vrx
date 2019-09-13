/*
 * Course To Drive
 * Purpose: Implements a course (speed, yaw) controller for the WamV
 * Inputs:
 *  - Speed: Desired speed
 *  - Yaw: Desired heading
 * Outputs: Motor Commands
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

class CourseToDrive
{
public:
    CourseToDrive()
	{
            // ros::NodeHandle n_("~");
            stbdPub_ = n_.advertise<std_msgs::Float32>("/thrusters/right_thrust_cmd", 1);
            portPub_ = n_.advertise<std_msgs::Float32>("/thrusters/left_thrust_cmd", 1);
            ros::param::get("~use_vel_fitting", use_vel_fitting_);
            ros::param::get("~negative_scaling_factor", negative_scaling_factor_);
            ros::param::get("~positive_scaling_factor", positive_scaling_factor_);
            // Get the relevant parameters
            ros::param::get("~lin_kp", lin_kp_);
            ros::param::get("~lin_ki", lin_ki_);
            ros::param::get("~lin_kd", lin_kd_);
            ros::param::get("~yaw_kp", yaw_kp_);
            ros::param::get("~yaw_ki", yaw_ki_);
            ros::param::get("~yaw_kd", yaw_kd_);

            ros::param::get("~motor_command_limit", motor_command_limit_);

            prev_lin_error_ = 0.0;
            acc_lin_error_ = 0.0;
            // max_lin_windup_ = 1.0;
            ros::param::get("~max_linear_windup", max_lin_windup_);
            ros::param::get("~max_yaw_windup", max_yaw_windup_);
            prev_yaw_error_ = 0.0;
            acc_yaw_error_ = 0.0;
            // max_ang_windup_ = 1.0;

            // The heartbeat is implemented to prevent the robot moving with an
            // out of date command. Default limmit is 1.0.
            heartbeat_timer_ = 0.;
            heartbeat_limit_ = 1.0;
            time_prev_ = ros::Time::now().toSec();

            // Start of with speed and yaw set to zero
            set_speed_ = 0.;
            set_yaw_ = 0.;

            // Don't do anything before a course command is received
            received_course_ = false;

            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("odom", 1, &CourseToDrive::localisationCallback, this);
            // Subscribe to course command
            courseSub_ =  n_.subscribe("cmd_course", 1, &CourseToDrive::courseCallback, this);
    }
    // Uses the odometry to get the current course, in order to find the difference
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& odom)
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
            quat.setValue(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;

            double alpha, velU, delta;
            double yaw_error, yaw_effort;
            double lin_error, lin_effort;

            // Calculate the difference between the desired and actual values
            yaw_error = CourseToDrive::constrainAngle(set_yaw_ - phi_);

            // Get the received twist message
            geometry_msgs::Twist twist;
            twist = odom->twist.twist;

            // Find forwards and sidewards velocity
            // velU = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
            double velM, velV, vel_angle, vel_yaw_diff;

            if (twist.linear.x < 0.0)
            {
                velU = -sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
            }
            else{
                velU = sqrt(pow(twist.linear.x,2) + pow(twist.linear.y,2));
            }
            // Linear Error
            lin_error = set_speed_ - velU;

            // Calculate the integral terms - do this separately to check for windup limits
            double yaw_iterm, lin_iterm;
            yaw_iterm = acc_yaw_error_ + time_diff*yaw_error;
            // Check the positive windup limit
            if(yaw_iterm > max_yaw_windup_)
            {
                yaw_iterm = max_yaw_windup_;
            }
            else if(yaw_iterm < -max_yaw_windup_) // Check the negative windup limit
            {
                yaw_iterm = -max_yaw_windup_;
            }
            // Calculat the linear i-term
            lin_iterm = acc_lin_error_ + time_diff*lin_error;
            // Check positive windup limit
            if(lin_iterm > max_lin_windup_)
            {
                lin_iterm = max_lin_windup_;
            }
            else if(lin_iterm < -max_lin_windup_) // Check negative windup limit
            {
                lin_iterm = -max_lin_windup_;
            }

            // A basic PID controller
            yaw_effort = yaw_kp_*yaw_error + yaw_ki_*yaw_iterm + yaw_kd_*(yaw_error - prev_yaw_error_)/time_diff;
            lin_effort = lin_kp_*lin_error + lin_ki_*lin_iterm + lin_kd_*(lin_error - prev_lin_error_)/time_diff;

            double Ts, Tp, Vs, Vp, vel_offset;
            // Velocity Offset = Dist Between Hulls * Angular Velocity Command
            vel_offset = 2.4384 * ( yaw_effort );

            Vs = ( lin_effort ) + vel_offset;
            Vp = ( lin_effort ) - vel_offset;

            // Calculate the saturation velocity, the maximum velocity the motors can achieve at the command limits.
            // These values are obtained from experiments, using GLF.
            double saturation_cmd_pos, saturation_cmd_neg, saturation_vel_pos, saturation_vel_neg;

            if(use_vel_fitting_)
            {
                // Calculate the saturation velocity, the maximum velocity the motors can achieve at the command limits.
                // These values are obtained from experiments, using GLF.
                saturation_vel_pos = command_to_vel(motor_command_limit_, 0.0, 0.31, 5.71, 0.65, 0.27, 0.18);
                saturation_vel_neg = command_to_vel(-motor_command_limit_, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
            }
            else
            {
                saturation_vel_pos = linear_vel_to_command(motor_command_limit_);
                saturation_vel_neg = linear_vel_to_command(-motor_command_limit_);
            }

            // The motors shouldn't be allowed to operate when saturated - with the GLF mapping it can output NaNs.
            bool within_saturation_limits = false;

            std::cout << "Vs: " << Vs << " Vp: " << Vp << " Offset: " << vel_offset << std::endl;

            // Sort the saturation, while prioritising the yaw rate over speed.
            while (!within_saturation_limits)
            {
                // Do the positive saturation velocities first - as these have higher saturation velocities
                if (Vs > saturation_vel_pos)
                {
                    // If it is over the saturation velocity, set to the maximum saturation value
                    // and adjust the other motor, so that the desired yaw_rate is obtained.
                    Vs = saturation_vel_pos-0.01;
                    Vp = Vs - 2*vel_offset;
                }
                if (Vp > saturation_vel_pos)
                {
                    // If it is over the saturation velocity, set to the maximum saturation value
                    // and adjust the other motor, so that the desired yaw_rate is obtained.
                    Vp = saturation_vel_pos-0.01;
                    Vs = Vp + 2*vel_offset;
                }
                // Do backwards last - as this has a smaller saturation value
                if (Vs < saturation_vel_neg)
                {
                    // If the value is below the maximum backwards velocity, set the maximum backwards velocity.
                    Vs = saturation_vel_neg+0.01;
                    // TODO check this.
                    if (vel_offset < saturation_vel_neg)
                    {
                        Vp = abs(Vs);
                    }
                    else
                    {
                        Vp = Vs - 2*vel_offset;
                    }
                }
                if (Vp < saturation_vel_neg)
                {
                    // If the value is below the maximum backwards velocity, set the maximum backwards velocity.
                    Vp = saturation_vel_neg+0.01;
                    if (vel_offset < saturation_vel_neg)
                    {
                        Vs = abs(Vp);
                    }
                    else
                    {
                        Vs = Vp + 2*vel_offset;
                    }
                }
                // Checks if the velocities are within targets
                if(Vs > saturation_vel_neg && Vs < saturation_vel_pos && Vp > saturation_vel_neg && Vs < saturation_vel_pos)
                {
                    within_saturation_limits = true;
                }
                else // If still not within limits - call the while loop off FIXME
                {
                    // It shouldn't get here, this is a last resort.
                    if (Vs < saturation_vel_neg)
                    {
                        Vs = saturation_vel_neg+0.01;
                    }
                    if (Vp < saturation_vel_neg)
                    {
                        Vp = saturation_vel_neg+0.01;
                    }
                    if (Vs > saturation_vel_pos)
                    {
                        Vs = saturation_vel_pos-0.01;
                    }
                    if (Vp > saturation_vel_pos)
                    {
                        Vp = saturation_vel_pos-0.01;
                    }
                    within_saturation_limits = true;
                }
            }
            // Convert velocity to thrust command - linearising the output
            if(Vs >= 0) // Use positive coefficients
            {
                if(use_vel_fitting_)
                {
                    // GLF+ Coefficients: A=0,K=0.31,B=5.71,v=0.65,C=0.27,M=0.18
                    Ts = vel_to_command(Vs, 0, 0.31, 5.71, 0.65, 0.27, 0.18);
                }
                else
                {
                    Ts = linear_vel_to_command(Vs);
                }

            }
            else // Use negative coefficients
            {
                if(use_vel_fitting_)
                {
                    // GLF- Coefficients: A=-1.35,K=0,B=4.89,v=0.05,C=1.00,M=-1.31
                    Ts = vel_to_command(Vs, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
                }
                else
                {
                    Ts = linear_vel_to_command(Vs);
                }

            }
            if(Vp >= 0)
            {
                if(use_vel_fitting_)
                {
                    // GLF+ Coefficients: A=0,K=0.31,B=5.71,v=0.65,C=0.27,M=0.18
                    Tp = vel_to_command(Vp, 0, 0.31, 5.71, 0.65, 0.27, 0.18);
                }
                else
                {
                    Tp = linear_vel_to_command(Vp);
                }

            }
            else
            {
                if(use_vel_fitting_)
                {
                    // GLF- Coefficients: A=-1.35,K=0,B=4.89,v=0.05,C=1.00,M=-1.31
                    Tp = vel_to_command(Vp, -1.35, 0.0, 4.89, 0.05, 1.00, -1.31);
                }
                else
                {
                    Tp = linear_vel_to_command(Vp);
                }
            }

            // std::cout << "Ts: " << Ts << " Tp: " << Tp << std::endl;

            // Create the output messages
            std_msgs::Float32 ts_msg;
            std_msgs::Float32 tp_msg;
            // Convert to float (from double)
            ts_msg.data = float(Ts);
            tp_msg.data = float(Tp);
            // ROS_DEBUG_STREAM("Ts: " << Ts << " Tp: " << Tp);
            // Publish to the motors
            stbdPub_.publish(ts_msg);
            portPub_.publish(tp_msg);

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
    // Receive the course message and store it
    void courseCallback(const vrx_msgs::Course::ConstPtr& msg)
    {
        if(ros::ok())
        {
            heartbeat_timer_ = 0.; //Reset the timer
            received_course_ = true; // Allow velocity commands to now be publisheds
            set_speed_ = msg->speed;
            set_yaw_ = msg->yaw;
        }
    }
    // Command to Thrust using GLF (Generalised Logistic Functions)
    float command_to_vel(float command, float A, float K, float B, float v, float C, float M)
    {
        // Uses GLF to map command to thrust
        float vel;
        vel = A + (K-A)/pow((C+exp(-B*(command -M))),1/v);
        return vel;
    }
    // Velocity to Command
    float vel_to_command(float vel, float A, float K, float B, float v, float C, float M)
    {
        // Equation:
        // command = \frac{ln(\frac{K-A}{vel-A}^v - C)}{-B} + M
        // Uses a rearranged GLF to map command to thrust
        float command;
        command = log((pow((K-A)/(vel - A),v) - C))/(-B) + M;
        return command;
    }
    // Linear scaling from command to velocity - used when vel fitting is false
    double linear_vel_to_command(double vel)
    {
        double cmd;
        if(vel < 0.0)
        {
            cmd = vel*negative_scaling_factor_;
        }
        else
        {
            cmd = vel*positive_scaling_factor_;
        }
        return cmd;
    }
    double linear_command_to_vel(double cmd)
    {
        double vel;
        if(cmd < 0.0)
        {
            vel = cmd*(1/negative_scaling_factor_);
        }
        else
        {
            vel = cmd*(1/positive_scaling_factor_);
        }
        return vel;
    }

private:
	ros::NodeHandle n_;
    ros::Subscriber subLocalise_;
    ros::Subscriber courseSub_;
    ros::Publisher velPub_;

    // Motor command publishers
    ros::Publisher stbdPub_;
    ros::Publisher portPub_;

    double phi_;

    // Controller gains and parameters
    double lin_kp_;
    double lin_kd_;
    double lin_ki_;
    double prev_lin_error_;
    double acc_lin_error_;
    double max_lin_windup_;

    double yaw_kp_;
    double yaw_kd_;
    double yaw_ki_;
    double prev_yaw_error_;
    double acc_yaw_error_;
    double max_yaw_windup_;

	double set_speed_;

    double set_yaw_;

    bool received_course_;

    double heartbeat_timer_;
    double heartbeat_limit_;
    double time_prev_;

    double motor_command_limit_;

    bool use_vel_fitting_;
    //Linear Scaling Factors
    double negative_scaling_factor_;
    double positive_scaling_factor_;
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "course_to_drive");

  CourseToDrive c2d;

  ros::spin();


  return 0;
}
