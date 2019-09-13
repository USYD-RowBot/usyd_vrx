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
import pyproj
params = {
    "inTopic": "/pointToHold",
    "outTopic": "/waypoints",
    "gpsTopic": "wamv/sensors/gps/gps/fix",
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

outProj = pyproj.Proj("+init=EPSG:4326")

def transformCoordinates(latpos,XYZpos):
    R = 6378100 #Radius of Earth Metres
    pi=3.14159265359
    crs="+proj=tmerc +lon_0={} +lat_0={} +units=m".format(-157.8901,21.30996)
    inp = pyproj.Proj(crs)
    y,x = pyproj.transform(outProj,inp,latpos.longitude,latpos.latitude)
    XYZpos.x=x
    XYZpos.y=y
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
    path = Path()
    path.header = data.header
    path.header.frame_id="map"# not sure why this isnt published :3
    poses = []
    for dps in data.poses:
        ps = PoseStamped()
        ps.header = dps.header
        ps.pose.orientation = dps.pose.orientation
        transformCoordinates(dps.pose.position,ps.pose.position)
        # move into map frame
        ps.header.frame_id='map'
        poses.append(ps)
    path.poses=poses
    pub.publish(path)
    print(path)

sat = rospy.Subscriber(params['gpsTopic'], NavSatFix, satcb)
sub = rospy.Subscriber(params['inTopic'], GeoPath, cb)
rospy.spin()
