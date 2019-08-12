/*
 * Navigate
 * Purpose: This node is designed to navigate the Kingfisher to specified waypoints
 * Inputs: Waypoints - an array of x,y waypoints
 * Outputs: Drive Messages which steer the Kingfisher, OR, PID setpoint and state commands
 * Method:
 * When waypoints are received, it manuevers to each of these waypoints, using Pure Pursuit, or NLGL
 * When the path is completeted, it requests new waypoints, while also stopping the motors.
 * If new waypoints are received before others are completed, the old are erased, and it begins the new.
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
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "vrx_msgs/Course.h"
#include "vrx_msgs/VesselWaypoint.h"
#include "vrx_msgs/VesselPath.h"

#include "tf/transform_datatypes.h"


//GEOS related includes
#include <geos/geom/Geometry.h>
#include <geos/geom/Coordinate.h>
#include <geos/geom/CoordinateArraySequence.h>
#include <geos/geom/LineSegment.h>


class Navigator
{
public:
    Navigator()
	{
            // Subscribe to EKF
            subLocalise_ =  n_.subscribe("odom", 1, &Navigator::localisationCallback, this);
            // Susbcribe to waypoints
            subWP_ =  n_.subscribe("waypoints", 1, &Navigator::wpCallback, this);
            // Publish waypoint request
            reqPub_ = n_.advertise<std_msgs::Bool>("request_waypoints", 1);
            // Course Publish
            coursePub_ = n_.advertise<vrx_msgs::Course>("cmd_course", 1);
            // Publish a message every time a waypoint is hit
            hitPub_ = n_.advertise<geometry_msgs::Pose>("waypoint_hit", 1);

            // The speed of the Kingfisher - Only for naive (P Controller)
            ros::param::get("~speed", speed_);  //n_.param("speed", speed_, 0.5);
            // The waypoint tolerance - the distance at which a waypoint activated
            ros::param::get("~tolerance", tolerance_);  //n_.param("waypoint_tolerance", tolerance_, 4.0);

            // Go - set to zero, as only goes when it has active waypoints
            go_ = false;
            // Use NLGL if true, use pure pursuit if false.
            ros::param::get("~use_nlgl", use_nlgl_);  //n_.param("use_nlgl", use_nlgl_, false);
            // The NLGL Radius
            ros::param::get("~nlgl_radius", nlgl_radius_); //n_.param("nlgl_radius", nlgl_radius_, 10.0);

            on_path_ = false;
	}
    // NLGL Line Following Control
    double nlglVTP(double L)
    {
        double xt, yt;
        double A, B, C, Det;

        if(tx_ == wx_)
        {
            A = 1;
            B = -2*py_;
            C = pow(py_,2) + pow(wx_, 2) - 2*px_*wx_ - pow(L,2);
            Det = pow(B, 2) - 4*A*C;
            if(Det>0)
            {
                double x1 = wx_;
                double x2 = wx_;
                double y1 = (-B + sqrt(Det))/(2*A);
                double y2 = (-B + sqrt(Det))/(2*A);
                double D1 = pow((ty_ - y1),2) + pow((tx_ - x1),2);
                double D2 = pow((ty_ - y2),2) + pow((tx_ - x2),2);
                if(D1<D2)
                {
                    xt = x1;
                    yt = y1;
                }
                else
                {
                    xt = x2;
                    yt = y2;
                }
                on_path_ = true;
            }
            else if(Det == 0)
            {
                xt = wx_;
                yt = (-B)/(2*A);
                on_path_ = true;
            }
            else
            {
                xt = wx_;
                yt = wy_;
            }
        }
        else
        {
            double m = (ty_ - wy_)/(tx_ - wx_);
            double cf = -m*wx_+wy_-py_;
            A = 1 + pow(m, 2);
            B = -2*px_+2*m*cf;
            C = pow(px_,2) - pow(L,2) + pow(cf, 2);
            Det = pow(B,2) - 4*A*C;
            if(Det > 0)
            {
                double x1 =(-B+sqrt(Det))/(2*A);
                double x2 = (-B-sqrt(Det))/(2*A);
                double y1 = m*x1-m*wx_+wy_;
                double y2 = m*x2-m*wx_+wy_;
                double D1 = pow((ty_ - y1),2) + pow((tx_ - x1),2);
                double D2 = pow((ty_ - y2),2) + pow((tx_ - x2),2);
                if(D1 < D2)
                {
                    xt = x1;
                    yt = y1;
                }
                else
                {
                    xt = x2;
                    yt = y2;
                }
                on_path_ = true;
            }
            else if(Det == 0)
            {
                xt = (-B)/(2*A);
                yt = m*xt - m*wx_+wy_;
                on_path_ = true;
            }
            else
            {
                // Not hitting the circle - define what to do
                //xt = wx_;
                //yt = wy_;
                geos::geom::Coordinate cW(wx_,wy_);
                geos::geom::Coordinate cT(tx_, ty_);
                geos::geom::Coordinate cP(px_, py_);
                geos::geom::LineSegment ls(cW, cT);
                geos::geom::Coordinate clp;
                ls.closestPoint(cP, clp);
                xt = clp.x;
                yt = clp.y;
            }
        }
        xt_ = xt;
        yt_ = yt;
    }
    // Pure Pursuit Control
    double purepursuit(double xt, double yt)
    {
        double theta;
        if(px_ == xt)
        {
            if(yt > py_)
            {
                theta = M_PI/2;
            }
            else
            {
                theta = -M_PI/2;
            }
        }
        else
        {
            theta = atan2(yt - py_, xt - px_);
        }
        return theta;
    }
    // Localisation Callback
    void localisationCallback(const nav_msgs::Odometry::ConstPtr& ekf)
	{
		if(ros::ok())
		{
            px_ = ekf->pose.pose.position.x;
            py_ = ekf->pose.pose.position.y;
            tf::Quaternion quat;
            quat.setValue(ekf->pose.pose.orientation.x, ekf->pose.pose.orientation.y, ekf->pose.pose.orientation.z, ekf->pose.pose.orientation.w);

            // the tf::Quaternion has a method to access roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            phi_ = yaw;

            // ROS_INFO("Pose - X: %lf, Y: %lf, Theta: %lf", px_, py_, phi_);

            if(go_)
            {
                // Select the waypoints
                selectWaypoint();

                if(waypointMissed())
                {
                    ROS_INFO("P(%f, %f) W(%f,%f) T(%f,%f)", px_, py_,  wx_, wy_, tx_, ty_);
                    // Request new waypoints, with complete false
                    requestWaypoints(false);
                    // Stop the vessel
                    stop_vessel();
                }

                if(use_nlgl_)
                {
                    nlglVTP(nlgl_radius_);
                }
                else
                {
                    xt_ = tx_;
                    yt_ = ty_;
                }
                double phid = purepursuit(tx_, ty_);
                //ROS_INFO("Direction: - tx: %lf ty: %lf phi: %lf", tx_, ty_, phid);
                //double phid = 0.3;
                // ROS_INFO("Pose - x: %lf y: %lf phi: %lf WP - tx: %lf ty: %lf idx: %d", px_, py_, phi_, tx_, ty_, wIdx_);
                // Publishes a course message
                vrx_msgs::Course course;
                course.speed = speed_;
                course.yaw = phid;
                coursePub_.publish(course);
            }
		}
    }

    // Stops the vessel
    void stop_vessel(void)
    {
        vrx_msgs::Course course;
        course.speed = 0.0;
        course.yaw = 0.0;
        coursePub_.publish(course);
    }

    // Waypoint Callback
    // Inputs a waypoint array and stores it. Tells the navigator to go.
    void wpCallback(const vrx_msgs::VesselPath::ConstPtr& vesselPath)
    {
        waypoints_.clear();
        // TODO add a waypoint of the current position at the start
        // e.g.:
        vrx_msgs::VesselWaypoint currP;
        currP.pose.position.x = px_;
        currP.pose.position.y = py_;
        currP.tolerance = tolerance_;
        waypoints_.push_back(currP);
        // If you do this then need to add +1 to numWP (wcount_)!!!

        int numWP = vesselPath->waypoints.size();
        wcount_ = numWP + 1; // +1 for the starting point
        wIdx_ = 0;
        float x, y;
        ROS_INFO("Number of Waypoints: %d", numWP);
        for(int n=0; n<numWP;n++)
        {
            waypoints_.push_back(vesselPath->waypoints.at(n));
        }

        go_ = true;
        on_path_ = false;  // This gets set to true when the robot moves onto the path.
        return;

     }
    // Requests new waypoints
    // Tell the planner that it has finished the waypoints
    void requestWaypoints(bool complete)
    {
        go_ = false;
        stop_vessel();
        std_msgs::Bool req;
        req.data = complete;
        reqPub_.publish(req);
        ROS_INFO("Requested new waypoints");
    }
    // Determine whether waypoint is missed
    bool waypointMissed()
    {
        // Setup a line segment of the waypoint
        geos::geom::Coordinate cW(wx_,wy_);
        geos::geom::Coordinate cT(tx_, ty_);
        geos::geom::Coordinate cP(px_, py_);
        geos::geom::LineSegment ls(cW, cT);
        // Find the projection factor
        double pf = ls.projectionFactor(cP);
        // If projection factor is greater than 1, the projection of the robot is
        // passed the waypoint path, meaning it has missed the waypoint.
        if(pf > 1.0)
        {
            // Return true if the waypoint is missed
            ROS_INFO("Missed Waypoint");
            return true;
        }
        // Return false if the waypoint is OK
        return false;
    }
    // Selects the appropriate waypoint
	void selectWaypoint()
	{
        double dist2wp = sqrt(pow((px_-tx_),2) + pow((py_-ty_),2));
        if(dist2wp < tolerance_)
        {
            // Publish a message indicating the waypoint is hit
            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = tx_;
            pose_msg.position.y = ty_;
            hitPub_.publish(pose_msg);
            // Increment the counter
            wIdx_++;
            if(wIdx_ >= wcount_)
            {
                wIdx_ = 0;
                requestWaypoints(true);
            }

            ROS_INFO("Waypoint Hit - tolerance = %f", tolerance_);
        }
        if(wcount_ > 0 && wIdx_+1 < wcount_)
        {
            vrx_msgs::VesselWaypoint wp = waypoints_.at(wIdx_);
            vrx_msgs::VesselWaypoint tp = waypoints_.at(wIdx_ + 1);
            wx_ = wp.pose.position.x;
            wy_ = wp.pose.position.y;
            tx_ = tp.pose.position.x;
            ty_ = tp.pose.position.y;
            tolerance_ = tp.tolerance;
        }

        ROS_INFO("P(%f, %f) W(%f,%f) T(%f,%f)", px_, py_,  wx_, wy_, tx_, ty_);

        return;
	}

private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
    ros::Subscriber subLocalise_;
    ros::Subscriber subWP_;
    ros::Publisher reqPub_;
    ros::Publisher hitPub_;

    ros::Publisher coursePub_;

	double speed_; // Set in establishment of the class

    float phi_;

	double wx_,wy_,tx_,ty_;

	double px_, py_;

    double xt_, yt_;

	double tolerance_;

	int wcount_;

    int wIdx_;

    bool go_;

    bool use_nlgl_;

    double nlgl_radius_;

    double kp_;

    std::vector<vrx_msgs::VesselWaypoint> waypoints_;

    bool on_path_;

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "navigator");

  Navigator nav;

  ros::spin();


  return 0;
}
