#!/usr/bin/env python

# Converts Geo Pose to Pose messages on required channel.
# Optional whether to just copy the values or copy them with appropriate scaling

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from vrx_msgs.msg import Waypoint
from vrx_msgs.msg import WaypointRoute
import tf
params = {
    "inTopic": "/waypoints",
    "outTopic": "/waypoints_cmd",
    "speed":1
}

rospy.init_node("pathToRoute",anonymous=True)

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

pub = rospy.Publisher(params['outTopic'], WaypointRoute, queue_size=1)
listener = tf.TransformListener()

def cb(data):
    print("RECEIVED WAYPOINT MESSAGE")
    global pub
    global listener
    route = WaypointRoute()
    waypoints = []
    for dps in data.poses:
        wp=Waypoint()
        wp.pose=dps.pose # it starts as a posestamped so extract the pose
        wp.nav_type=wp.NAV_STATION
        wp.station_duration = 3 # Wait 3 seconds at station to minimise error
        waypoints.append(wp)
    route.waypoints=waypoints
    route.speed=params["speed"]
    pub.publish(route)

sub = rospy.Subscriber(params['inTopic'], Path, cb)
rospy.spin()
