#!/usr/bin/env python
# Author: Jackson Shields


import sys
import argparse
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point
from std_msgs.msg import Float64, Bool, Float32
import math
import tf
from rospy.numpy_msg import numpy_msg

import shapely
import shapely.geometry

from rowbot_msgs.msg import Course

from path_following import NLGLvtp, purepursuit

from waypoint_passed import waypoint_passed

class CircleFollowing():
    """
    This class performs basic station keeping for a WAM-V.

    Motivated by conversation with ASV global:
        "How do you do you perform station keeping":
    Answer:
        There are two modes:
        - Option 1: If it is within a given tolerance of the target - does nothing
                    otherwise drives to the station.
        - Option 2: Set a circle and it slowly follows this circle.

    This class is Option 2.
    """

    def __init__(self):
        """Initiates the CircleFollowing Class

        Args:

        Returns:
            type: None

        """
        robot_name = rospy.get_param("~robot_name", "")
        # # Map the namespace into the topics
        if robot_name is None or robot_name == "":
            course_topic = '/cmd_course'
            odom_topic = '/odometry/filtered'
            station_topic = '/cmd_station'
        else:
            course_topic = '/' + robot_name + '/cmd_course'
            odom_topic = '/' + robot_name + '/odometry/filtered'
            station_topic = '/' + robot_name + '/cmd_station'
        # The publishers
        self.coursePub_ = rospy.Publisher(course_topic, Course, queue_size=1)
        # Go - if go is 0, don't move, if 1, then go.
        self.go = False
        # Pose
        self.px = 0.
        self.py = 0.
        self.phi = 0.
        # Tolerance to hit the waypoints
        self.radius = rospy.get_param("~radius", 10.0)  # If speed control is on, this is max speed
        self.nlgl_radius = rospy.get_param("nlgl_radius", 4.0)
        self.max_speed = rospy.get_param("~max_speed", 1.0) # Traverse speed
        self.min_speed = rospy.get_param("~min_speed", 0.5) # Hold speed
        self.direction = rospy.get_param("~direction", "CCW")

        # Prefer adding the subscribers at the end of init
        rospy.Subscriber(odom_topic, Odometry, self.localisationCallback)
        rospy.Subscriber(station_topic, Point, self.stationCallback)

        print("End of Init")


    def localisationCallback(self, data):
        """ Receives the localisation data and navigates the vessel

        Args:
            data (Odometry): An odometry message

        Returns:
            None

        """
        if self.go:
            self.px = data.pose.pose.position.x
            self.py = data.pose.pose.position.y
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            self.phi = euler[2]

            # Get the distance to the station centre
            distStation = np.sqrt((self.px - self.station.x)**2 + (self.py - self.station.y)**2)
            angleStation = math.atan2(self.station.y - self.py, self.station.x - self.px)

            self.stationShape = shapely.geometry.Point(self.station.x, self.station.y).buffer(self.radius)
            robotShape = shapely.geometry.Point(self.px, self.py).buffer(self.nlgl_radius)
            intersections = np.array(robotShape.boundary.intersection(self.stationShape.boundary))
            if intersections is None or intersections.size == 0:
                if distStation < self.radius:
                    # Contained within, just go straight until you hit circle -  #TODO improve???
                    set_speed = self.min_speed
                    set_yaw = self.phi
                else:
                    # Out of range, aim at the centre, go max speed
                    set_speed = self.max_speed
                    set_yaw = angleStation
            else:
                # Make sure intersections is shaped correctly, as 2D vector
                intersections = intersections.reshape(-1, 2)
                # print(intersections)
                # There is an intersection between NLGL circle and station circle
                if intersections.shape[0] == 1:
                    set_yaw = math.atan2(intersections[0,1] - self.py, intersections[0,0] - self.px)
                    set_speed = self.min_speed
                else:
                    vtp = np.zeros([2])
                    # There should be two angles now
                    angle_from_centre0 = math.atan2(intersections[0, 1] - self.station.y, intersections[0,0] - self.station.x)
                    angle_from_centre1 = math.atan2(intersections[1, 1] - self.station.y, intersections[1,0] - self.station.x)
                    angle_diff = np.unwrap(np.array([angle_from_centre1 - angle_from_centre0]))
                    if self.direction == "CW":
                        # Aim for the negative diff angle
                        if angle_diff > 0:
                            vtp = intersections[0,:]
                        else:
                            vtp = intersections[1,:]
                    else:
                        # Counter clockwise, aim for positive diff angle
                        if angle_diff > 0:
                            vtp = intersections[1,:]
                        else:
                            vtp = intersections[0,:]
                    set_speed = self.min_speed
                    set_yaw = math.atan2(vtp[1] - self.py, vtp[0] - self.px)

            rospy.loginfo("Position: %f,%f Station: %f,%f Speed: %f Yaw: %f" %(self.px, self.py, self.station.x, self.station.y, set_speed, set_yaw))
            course_msg = Course()
            course_msg.speed = set_speed
            course_msg.yaw = set_yaw
            self.coursePub_.publish(course_msg)


    def stationCallback(self, point_msg):
        """ Receives the waypoint array from the waypoint publisher

        Args:
            point_msg (Point): A point at which to keep station at
        Returns:
            None

        """
        # Delete all the old waypoints
        self.go = True
        self.station = point_msg
        rospy.loginfo("Received station command")


if __name__ == '__main__':
    """
    Navigates the WamV to a set of waypoints
    """

    rospy.init_node('circle_following', anonymous=True)

    cf = CircleFollowing()

    rospy.spin()
