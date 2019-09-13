#!/usr/bin/env python
# Author: Jackson Shields


import sys
import argparse
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point
from std_msgs.msg import Float64, Bool, Float32
import math
import tf
from rospy.numpy_msg import numpy_msg

import shapely

from vrx_msgs.msg import Course

from path_following import NLGLvtp, purepursuit

from waypoint_passed import waypoint_passed

class WamNav():
    """
    This class navigates the WamV in the uuv_simulator
    """

    def __init__(self):
        """Initiates the navigator class, with a user specified speed,
        following type and waypoint hitting tolerance

        Args:
            speed (float): The forwards velocity of the vessel in m/s
            use_nlgl (bool): Whether to use Non-Linear Guidance Law path following
            tolerance (float): The tolerance to hit a waypoint (if the distance
            to the waypoint is less, the waypoint will be considered hit.)

        Returns:
            type: None

        """
        robot_name = rospy.get_param("~robot_name", "")
        # # Map the namespace into the topics
        if robot_name is None or robot_name == "":
            course_topic = '/cmd_course'
            request_topic = '/request_waypoints'
            odom_topic = '/wamv/robot_localization/odometry/filtered'
            waypoint_topic = '/vrx/wayfinding/waypoints'
            vtp_topic = '/target_point'
        else:
            course_topic = '/' + robot_name + '/cmd_course'
            request_topic = '/' + robot_name + '/request_waypoints'
            odom_topic = '/' + robot_name + '/robot_localization/odometry/filtered'
            waypoint_topic = '/' + robot_name + '/waypoints'
            vtp_topic = '/' + robot_name + '/target_point'
        # The publishers
        self.wpRequestPub_ = rospy.Publisher(request_topic, Bool, queue_size=1)
        self.coursePub_ = rospy.Publisher(course_topic, Course, queue_size=1)
        self.vtpPub_ = rospy.Publisher(vtp_topic, Point, queue_size=1)
        # Go - if go is 0, don't move, if 1, then go.
        self.go = False
        # Pose
        self.px = 0.
        self.py = 0.
        self.phi = 0.
        # Waypoints
        self.wx = 0.
        self.wy = 0.
        self.tx = 0.
        self.ty = 0.
        # These are kept in self for logging (not really necessary)
        self.distwp = 0.
        self.wpAngle = 0.
        # Holds the waypoints list
        self.waypoints = []
        self.wcount = 0.
        self.wIdx = 0.
        # On Track variable - set true when it is put on the right path
        self.on_track = False
        # Tolerance to hit the waypoints
        self.speed = rospy.get_param("~speed", 1.0)  # If speed control is on, this is max speed
        self.use_nlgl = rospy.get_param("~use_nlgl", True)
        self.tolerance = rospy.get_param("~tolerance", 20.0)
        self.nlgl_radius = rospy.get_param("~nlgl_radius", 4.0)
        self.speed_control = rospy.get_param("~speed_control", False)
        self.braking_distance = rospy.get_param("~braking_distance", 4.0)
        self.minimum_forwards_speed = rospy.get_param("~minimum_forwards_speed", 0.1)
        # Hard Mode: Will hit waypoints regardless
        # Soft Mode: Requests new waypoints if exceeds a waypoint, or goes further from the path than allowed
        waypoint_mode = rospy.get_param("request_mode", "hard")
        if waypoint_mode == "hard":
            self.waypoint_hard_mode = True
        else:
            self.waypoint_hard_mode = False
        self.allowable_distance_to_path = rospy.get_param("~allowable_distance_to_path", 6.0)
        # Prefer adding the subscribers at the end of init
        rospy.Subscriber(odom_topic, Odometry, self.localisationCallback)
        rospy.Subscriber(waypoint_topic, Path, self.waypointCallback)

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

            self.waypointSelect(self.tolerance)
            if not self.waypoint_hard_mode:
                self.check_path()
            self.wpAngle = math.atan2(self.ty - self.wy, self.tx - self.wx)
            if self.speed_control:
                controlled_speed = self.control_speed(self.dist2wp, self.wpAngle)
            else:
                controlled_speed = self.speed


            if self.use_nlgl:
                [xt, yt] = NLGLvtp(self.wx, self.wy, self.tx, self.ty, self.px, self.py, self.nlgl_radius)
                #print xt, yt
                phid = purepursuit(xt, yt, self.px, self.py)
            else:
                phid = purepursuit(self.tx, self.ty, self.px, self.py)
                xt = self.tx
                yt = self.ty

            # phid is the angle to the target waypoint
            # Stands for 'phi desired'
            course_msg = Course()
            course_msg.speed = controlled_speed
            course_msg.yaw = phid
            self.coursePub_.publish(course_msg)

            # Publish the target point - not used - mainly for debugging Nav
            tp_msg = Point()
            tp_msg.x = xt
            tp_msg.y = yt
            tp_msg.z = 0.
            self.vtpPub_.publish(tp_msg)

    def waypointCallback(self, path_msg):
        """ Receives the waypoint array from the waypoint publisher

        Args:
            pose_array (PoseArray): A 6D pose array of the waypoints (only x,y are used)

        Returns:
            None

        """
        # Delete all the old waypoints
        self.waypoints = []
        # Append the current position
        self.waypoints.append(np.array([self.px, self.py]))
        for pose in path_msg.poses:
            self.waypoints.append(np.array([pose.pose.position.x, pose.pose.position.y]))
        self.wIdx = 0
        self.wcount = len(self.waypoints)
        # Set the targets here - avoids it hitting waypoint select with the wrong targets
        self.tx = self.waypoints[self.wIdx + 1][0]
        self.ty = self.waypoints[self.wIdx + 1][1]
        self.go = True
        self.on_track = False
        print("Received waypoints - commencing")

    def requestWaypoints(self):
        """ Publishes a message saying the current waypoints are finished,
        and more are needed.

        Returns:
            None

        """
        self.go = False
        self.on_track = False
        course_msg = Course()
        course_msg.speed = 0.0
        course_msg.yaw = self.phi # Keep the current phi
        self.coursePub_.publish(course_msg)
        req = Bool()
        req.data = True
        self.wpRequestPub_.publish(req)
        print("Requesting new waypoints")

    def waypointSelect(self, tolerance):
        """ Tests if the waypoint has been hit, then selects the waypoint

        Args:
            tolerance (float): If the distance to the waypoint is less than this,
            it is considered hit

        Returns:
            None

        """
        self.dist2wp = math.sqrt((self.ty - self.py)**2 + (self.tx - self.px)**2)
        #print dist2wp
        if(self.dist2wp < tolerance):
            print("Waypoint hit @ x:%f y:%f with x:%f y:%f" %(self.tx, self.ty, self.px, self.py))
            self.wIdx = self.wIdx + 1
            print("Index: %d Count: %d" %(self.wIdx, self.wcount))
            if(self.wIdx >= (self.wcount-1)):
                self.requestWaypoints()
                return

        self.tx = self.waypoints[self.wIdx + 1][0]
        self.ty = self.waypoints[self.wIdx + 1][1]
        self.wx = self.waypoints[self.wIdx][0]
        self.wy = self.waypoints[self.wIdx][1]
        self.dist2wp = math.sqrt((self.ty - self.py)**2 + (self.tx - self.px)**2)


    def _dist_to_path(self):
        """Calculates the distance of the vessel from the waypoint path

        Returns
        -------
        Float
            Distance to waypoint path

        """
        m = (self.ty - self.wy)/(self.tx - self.wx)
        return math.fabs(m*self.px - 1 + self.py)/(math.sqrt(m**2 + 1))

    def check_path(self):
        """Checks if the robots path is ok

        Returns:
            float: If the robot position is ok for the path, it return true.
            If it is false, the robots path is bad, and  a new path is needed.

        """
        path_ok = True
        dist2path = self._dist_to_path()
        wp_passed = waypoint_passed([self.wx, self.wy], [self.tx, self.ty], [self.px, self.py])

        if dist2path < self.allowable_distance_to_path and not wp_passed:
            self.on_track = True
            path_ok = True
        elif dist2path > self.allowable_distance_to_path or wp_passed:
            if self.on_track:
                # Now the path is invalid - request new waypoints
                path_ok = False
        if not path_ok:
            print("Requesting new path")
            self.requestWaypoints()

    def control_speed(self, dist_to_wp, wp_angle):
        """Applies braking before corners. #TODO when angle away from waypoint line.

        Args:
            dist_to_wp (float): Distance to the waypoint

        Returns:
            float: speed of the vehicle
        """

        if dist_to_wp < self.braking_distance:
            # Get next waypoint
            if(self.wIdx < (self.wcount-2)): # Checks if this isn't the last waypoint
                nx = self.waypoints[self.wIdx + 2][0]
                ny = self.waypoints[self.wIdx + 2][1]
                theta = wp_angle
                ntheta = math.atan2(ny - self.ty, nx - self.tx)
                diff = np.unwrap(np.array([ntheta - theta]))
                speed_reduct_factor = (np.pi - np.abs(diff))/np.pi
                wp_controlled_speed = self.speed*speed_reduct_factor
                # print("Controlling Speed")
            else:
                wp_controlled_speed = self.speed # If it is the last waypoint, set to max
        else:
            wp_controlled_speed = self.speed

        heading_diff = np.abs(np.unwrap(np.array([self.phi - wp_angle])))

        uhl = np.pi/2  # Upper heading limit
        lhl = np.pi/6  # Lower heading limit

        if heading_diff > uhl:  # If greater 90degrees, minimum speed
            heading_controlled_speed = self.minimum_forwards_speed
        elif heading_diff < lhl:  # If heading difference is small, go full speed
            heading_controlled_speed = self.speed
        else:
            heading_factor = ((uhl - lhl) - (heading_diff - lhl))/(uhl - lhl)
            heading_controlled_speed = (self.speed - self.minimum_forwards_speed)*heading_factor + self.minimum_forwards_speed
            if heading_controlled_speed > self.speed:
                heading_controlled_speed = self.speed
            elif heading_controlled_speed < self.minimum_forwards_speed:
                heading_controlled_speed = self.minimum_forwards_speed

        controlled_speed = min([wp_controlled_speed, heading_controlled_speed])
        # TODO Review this
        if controlled_speed < self.minimum_forwards_speed:
            controlled_speed = self.minimum_forwards_speed
        elif controlled_speed > self.speed:
            controlled_speed = self.speed

        return controlled_speed



if __name__ == '__main__':
    """
    Navigates the WamV to a set of waypoints
    """

    rospy.init_node('wamv_navigator', anonymous=True)

    hnav = WamNav()

    rospy.spin()
