#!/usr/bin/env python

# Converts VRX Objects to Geo Pose messages on required topic

import rospy
import pyproj
import tf
from vrx_msgs.msg import ObjectArray
from geographic_msgs.msg import GeoPoseStamped

params = {
    "inTopic": "/objects",
    "outTopic": "/vrx/perception/landmark",
}
rospy.init_node("perceptionConverter",anonymous=True)

listener = tf.TransformListener()

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
    "surmark46104":"surmark46104",
    "surmark950400":"surmark950400",
    "surmark950410":"surmark950410"
}


inFormat = pyproj.Proj("+init=EPSG:4326")
zeroMerc=pyproj.Proj("+proj=tmerc +lon_0={} +lat_0={} +units=m".format(-157.8901,21.30996))
def convertToLatLon(pose):
    lon,lat = pyproj.transform(zeroMerc,inFormat,pose.x,pose.y)
    return {"latitude":lat,"longitude":lon,'altitude':0}

def cb(data):
    for i in data.objects:
        #Create a new geopose to publish
        gp=GeoPoseStamped()
        gp.header.stamp=rospy.Time.now()
        gp.header.frame_id=allowed_strings[i.best_guess]
        latlon=convertToLatLon(data.position.pose)
        gp.pose.orientation=i.pose.orientation
        gp.pose.position.latitude=latlon['latitude']
        gp.pose.position.longitude=latlon['longitude']
        gp.pose.position.altitude=latlon['altitude']
        pub.publish(gp)



sub = rospy.Subscriber(params['inTopic'], ObjectArray, cb)
rospy.spin()
