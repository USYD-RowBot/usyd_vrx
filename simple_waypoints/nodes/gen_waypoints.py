#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008 Willow Garage

# Author: Jackson Shields
#  The purpose of this node is to follow a survey path
import sys
import argparse
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import pymap3d
import json

class PublishWaypoints():
    """Publishes waypoints from csv,json for robots to follow

    Params:
        filename (str): The file containing the waypoints
        namespace (str): The namespace of to publish waypoints to
        geo (bool): Whether the coordinates are in Geodetic form
        recycle (bool): Whether to recycle waypoints
    """
    def __init__(self):
        filename = rospy.get_param("~filename")
        namespace = rospy.get_param("~namespace", "")
        geo = rospy.get_param("~geo")
        recycle = rospy.get_param("~recycle", False)
        if namespace is None or namespace == "":
            waypoint_topic = '/waypoints'
            request_topic = '/request_waypoints'
        else:
            waypoint_topic = '/' + namespace + '/waypoints'
            request_topic = '/' + namespace + '/request_waypoints'
        self.wpPub_ = rospy.Publisher(waypoint_topic, Path, queue_size=1, latch=True)
        rospy.Subscriber(request_topic, Bool, self.reqCallback)

        self.geo = geo
        self.recycle = recycle

        if filename.endswith('.json'):
            self.get_from_geojson(filename)
        elif filename.endswith('.csv'):
            self.get_from_csv(filename)
        else:
            print("File not supported")


    def reqCallback(self, request):
        """Callback for the topic that requests new waypoints

        Args:
            request (bool): Placeholder

        Returns:
            None

        """
        print("Request received")
        if(self.recycle):
            self.get_from_csv(self.csvfile)
        else:
            print("Survey Complete")

    def get_from_csv(self, filename):
        """ Reads waypoints from a .csv file

        Args:
            filename (str): The CSV file

        Returns:
            None

        """
        waypoint_array = np.genfromtxt(filename, delimiter=",")
        if(self.geo):
            self.publish_waypoints(self.geo_to_ENU(waypoint_array))
        else:
            self.publish_waypoints(waypoint_array)

    def get_from_geojson(self, filename):
        """ Loads the waypoints from a geojson

        Args:
            filename (str): The geojson file

        Returns:
            None

        """
        with open(filename, "r") as f:
            geo_dict = json.load(f)
        for feature in geo_dict['features']:
            wp_list = feature['geometry']['coordinates']
        waypoint_array = np.asarray(wp_list)
        # GEOJSON always in geodetic coordinates
        self.publish_waypoints(self.geo_to_ENU(waypoint_array))

    def geo_to_ENU(self, wpointsGEO):
        """Converts geodetic waypoints to ENU

        Args:
            wpointsGEO (Nx3 numpy array): a numpy array of geodetic coordinates

        Returns:
            Nx2 numpy array: ENU coordinates

        """
        lat0 = rospy.get_param('datum_latitude')
        long0 = rospy.get_param('datum_longitude')
        alt0 = rospy.get_param('datum_altitude')

        waypointsENU = np.zeros([np.size(wpointsGEO, 0), 2])
        for n in range(np.size(wpointsGEO,0)):
            [E, N, U] = pymap3d.geodetic2enu(wpointsGEO[n, 0], wpointsGEO[n, 1], alt0, lat0, long0, alt0)
            # IF SIM
            waypointsENU[n, 0] = E
            waypointsENU[n, 1] = N

        return waypointsENU

    def publish_waypoints(self, waypoint_array):
        """Publishes the waypoints so the

        Args:
            waypoint_array (Nx2 numpy array): The waypoints to be published

        Returns:
            None

        """
        path_msg = Path()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        path_msg.header = header
        pose_list = []
        for n in range(waypoint_array.shape[0]):
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = waypoint_array[n, 0]
            ps.pose.position.y = waypoint_array[n, 1]
            pose_list.append(ps)
        path_msg.poses = pose_list
        self.wpPub_.publish(path_msg)


if __name__ == '__main__':
    rospy.init_node('waypoint_publish', anonymous=True)

    # parser = argparse.ArgumentParser(
    #     description="Publishes a CSV or GEOJSON as a PoseArray message")
    # parser.add_argument(
    #     "filename", help="Path to the CSV or GEOJSON containing the waypoints")
    # parser.add_argument("--namespace", help="The namespace to publish waypoints in")
    # parser.add_argument("--geo", action='store_true', help="Are the waypoints stored in geodetic form?")
    # parser.add_argument("--recycle", action='store_true', help="When the waypoints are finished")
    #
    # args = parser.parse_args(rospy.myargv()[1:])

    wayPub = PublishWaypoints()

    rospy.spin()
