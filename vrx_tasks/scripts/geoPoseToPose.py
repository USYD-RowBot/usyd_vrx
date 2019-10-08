#!/usr/bin/env python

# Converts Geo Pose to Pose messages on required topic.

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
import tf
import pyproj
params = {
    "inTopic": "/vrx/station_keeping/goal",
    "outTopic": "/station",
    "gpsTopic": "wamv/sensors/gps/gps/fix",
    "toConvert": True
}
rospy.init_node("geoPoseToPose",anonymous=True)
for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

pub = rospy.Publisher(params['outTopic'], PoseStamped, queue_size=1)
lastSatPos=None
listener = tf.TransformListener()

def satcb(data):
    global lastSatPos
    lastSatPos=data

outProj = pyproj.Proj("+init=EPSG:4326")

def transformCoordinates(latpos,XYZpos):
    crs="+proj=tmerc +lon_0={} +lat_0={} +units=m".format(-157.8901,21.30996)
    inp = pyproj.Proj(crs)
    y,x = pyproj.transform(outProj,inp,latpos.longitude,latpos.latitude)
    XYZpos.x=y
    XYZpos.y=x
    XYZpos.z=0


def cb(data):
    print("RECEIVED GEO PATH MESSAGE")
    global pub
    global lastSatPos
    global listener
    print (data)
    if lastSatPos is None:
        print("NO SAT DATA, FAIL")
        return
    print (lastSatPos)
    ps = PoseStamped()
    ps.header = data.header
    ps.pose.orientation = data.pose.orientation
    transformCoordinates(data.pose.position,ps.pose.position)
    # move into map frame
    ps.header.frame_id='map'
    pub.publish(ps)

sat = rospy.Subscriber(params['gpsTopic'], NavSatFix, satcb)
sub = rospy.Subscriber(params['inTopic'], GeoPoseStamped, cb)
rospy.spin()