#!/usr/bin/env python

# Converts Geo Pose to Pose messages on required channel.
# Optional whether to just copy the values or copy them with appropriate scaling

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
params = {
    "inTopic": "/pointToHold",
    "outTopic": "/waypoints",
    "gpsTopic": "/GPS",
    "toConvert": True
}
rospy.init_node("geoPathToPath",anonymous=True)
for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

pub = rospy.Publisher(params['outTopic'], Path)
lastSatPos=None
listener = tf.TransformListener()

def satcb(data):
    global lastSatPos
    lastSatPos=data

def cb(data):
    print("RECEIVED GEO PATH MESSAGE")
    global pub
    global lastSatPos
    global listener
    if lastSatPos is None:
        print("NO SAT DATA, FAIL")
        return
    path = Path()
    path.header = data.header
    poses = []
    for dps in data.poses:
        ps = PoseStamped()
        ps.header = dps.header
        ps.pose.orientation = dps.pose.orientation
        
        # convert to relative coordinates
        ps.pose.position.x = (dps.pose.position.latitude-lastSatPos.latitude) * \
            (1000 if (params['toConvert']) else 1)
        ps.pose.position.y = (dps.pose.position.longitude-lastSatPos.longitude) * \
            (1000 if (params['toConvert']) else 1)
        ps.pose.position.z = dps.pose.position.altitude-lastSatPos.altitude

        # move into map frame
        ps.header.frame_id='map'
        relative=listener.lookupTransform('/base_link','/map')
        ps.pose.position.x-=relative.position.x
        ps.pose.position.y-=relative.position.y
        ps.pose.position.z-=relative.position.z

        poses.append(ps)
    path.poses=poses
    print(path)
    pub.publish(path)

sat = rospy.Subscriber(params['gpsTopic'], NavSatFix, satcb)
sub = rospy.Subscriber(params['inTopic'], GeoPath, cb)
rospy.spin()