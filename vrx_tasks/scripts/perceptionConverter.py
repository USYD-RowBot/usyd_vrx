#!/usr/bin/env python

# Converts VRX Objects to Geo Pose messages on required topic

import rospy
from vrx_msgs.msg import ObjectArray
from geographic_msgs.msg import GeoPoseStamped
params = {
    "inTopic": "/objects",
    "outTopic": "/vrx/perception/landmark",
    "gpsTopic": "wamv/sensors/gps/gps/fix",
}
rospy.init_node("geoPoseToPose",anonymous=True)

for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

pub = rospy.Publisher(params['outTopic'], GeoPoseStamped)

allowed_strings={
    "yellow_totem":"yellow_totem",
    "black_totem":"black_totem",
    "blue_totem":"blue_totem",
    "green_totem":"green_totem",
    "red_totem":"red_totem",
    "polyform_a3":"polyform_a3",
    "polyform_a5":"polyform_a5",
    "polyform_a7":"polyform_a7",
    "surmark_46104":"surmark_46104",
    "surmark_950400":"surmark_950400",
    "surmark_950410":"surmark_950410"
}

def cb(data):
    for i in data.objects:
        #Create a new geopose to publish
        gp=GeoPoseStamped()
        gp.header.stamp=rospy.Time.now()
        gp.header.frame_id=allowed_strings[i.best_guess]
        latlon=convertToLatLon(pose)
        gp.pose.orientation=i.pose.orientation
        gp.pose.position.latitude=latlon['latitude']
        gp.pose.position.longitude=latlon['longitude']
        gp.pose.position.altitude=latlon['altitude']




from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import tf
import pyproj


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