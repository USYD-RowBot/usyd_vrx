#!/usr/bin/env python

# Converts pose messages to course messages on specified topic.
# Optional whether to just copy the values or copy them with appropriate scaling

import rospy
from geometry_msgs.msg import PoseStamped
from vrx_msgs.msg import Waypoint
from vrx_msgs.msg import WaypointRoute
#import tf

class poseToCourseConverter:
    def __init__(self):
        params = {
            "inTopic": "/station",
            "outTopic": "/waypoints_cmd",
            "speed":1
        }
        rospy.init_node("poseToCourse",anonymous=True)
        for i in params:
            params[i] = rospy.get_param('~'+i, params[i])
        self.pub = rospy.Publisher(params['outTopic'], WaypointRoute, queue_size=1)
        #listener = tf.TransformListener()
        rospy.Subscriber(params['inTopic'], PoseStamped, self.cb)
        self.speed=params['speed']

    def cb(self,data):
        print("RECEIVED WAYPOINT MESSAGE")
        route = WaypointRoute()
        waypoints = []

        wp=Waypoint()
        wp.pose=data.pose   
        wp.nav_type=wp.NAV_STATION
        wp.station_duration = -1   # Stay at station until received new command

        waypoints.append(wp)
        
        route.waypoints=waypoints
        route.speed=self.speed
        self.pub.publish(route)