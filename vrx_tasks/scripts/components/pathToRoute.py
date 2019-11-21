#!/usr/bin/env python

# Converts Geo Pose to Pose messages on required channel.
# Optional whether to just copy the values or copy them with appropriate scaling

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from vrx_msgs.msg import Waypoint
from vrx_msgs.msg import WaypointRoute
#import tf

class pathToRouteConverter:
    def __init__(self):
        params = {
            "inTopic": "/waypoints",
            "outTopic": "/waypoints_cmd",
            "speed":5
        }

        #rospy.init_node("pathToRoute",anonymous=True)

        for i in params:
            params[i] = rospy.get_param('~'+i, params[i])

        self.pub = rospy.Publisher(params['outTopic'], WaypointRoute, queue_size=1)
        #self.listener = tf.TransformListener()
        self.speed=params['speed']
        rospy.Subscriber(params['inTopic'], Path, self.cb)


    def cb(self,data):
        #print("RECEIVED WAYPOINT MESSAGE")
        route = WaypointRoute()
        waypoints = []
        for dps in data.poses:
            wp=Waypoint()
            wp.pose=dps.pose # it starts as a posestamped so extract the pose
            wp.nav_type=wp.NAV_STATION
            wp.station_duration = 6 # Wait for seconds at station to minimise error
            waypoints.append(wp)

        waypoints[-1].station_duration = -1.0 # Make last waypoint infinite

        route.waypoints=waypoints
        route.speed=self.speed
        self.pub.publish(route)