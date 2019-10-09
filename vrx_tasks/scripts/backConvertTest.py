#!/usr/bin/env python

# Test driver for perception converter

# import pyproj

# inFormat = pyproj.Proj("+init=EPSG:4326")
# outFormat = pyproj.Proj("+proj=tmerc")
# zeroMerc=pyproj.Proj("+proj=tmerc +lon_0={} +lat_0={} +units=m".format(-157.8901,21.30996))
# y,x = pyproj.transform(inFormat,zeroMerc,-157.8901,21.30996)
# print y,x
# # USE THE FOLLOWING LINE: REPLACE 10,10 with your coordinates!
# y,x = pyproj.transform(zeroMerc,inFormat,10,10)
# print y,x


import rospy
import pyproj
import tf
from vrx_msgs.msg import ObjectArray
from geographic_msgs.msg import GeoPoseStamped

params = {
    "outTopic": "/objects",
}
rospy.init_node("perceptionConverterTest",anonymous=True)

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
    "surmark_46104":"surmark_46104",
    "surmark_950400":"surmark_950400",
    "surmark_950410":"surmark_950410"
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