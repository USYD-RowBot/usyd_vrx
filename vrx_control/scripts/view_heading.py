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


class ViewHeading():
    """
    This class prints the heading
    """

    def __init__(self):
        """Prints the headings
        """
        robot_name = rospy.get_param("~robot_name", "")
        odom_topic = '/odom'
        # The publishers
        rospy.Subscriber(odom_topic, Odometry, self.localisationCallback)
    
    def localisationCallback(self, data):
        """ Receives the localisation data and navigates the vessel

        Args:
            data (Odometry): An odometry message

        Returns:
            None

        """
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
        print("Heading: %f" %euler[2])

if __name__ == '__main__':
    """
    Navigates the WamV to a set of waypoints
    """

    rospy.init_node('view_heading', anonymous=True)

    vh = ViewHeading()

    rospy.spin()
