#!/usr/bin/env python

# Converts Geo Pose to Pose messages on required topic.

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
#import tf
import pyproj


class geoPoseToPoseConverter:
    def __init__(self):
        rospy.loginfo("Initalizing geoPosetoPose converter")
        params = {
            "inTopic": "/vrx/station_keeping/goal",
            "outTopic": "/station",
            "gpsTopic": "wamv/sensors/gps/gps/fix",
            "toConvert": True
        }
        #rospy.init_node("geoPoseToPose",anonymous=True)
        for i in params:
            params[i] = rospy.get_param('~'+i, params[i])
        self.pub = rospy.Publisher(params['outTopic'], PoseStamped, queue_size=1)
        self.lastSatPos=None
        #self.listener = tf.TransformListener()
        self.outProj = pyproj.Proj("+init=EPSG:4326")
        rospy.Subscriber(params['gpsTopic'], NavSatFix, self.satcb)
        rospy.Subscriber(params['inTopic'], GeoPoseStamped, self.cb)

    def satcb(self, data):
        self.lastSatPos=data

    def transformCoordinates(self,latpos,XYZpos):
        crs="+proj=tmerc +lon_0={} +lat_0={} +units=m".format(-157.8901,21.30996)
        inp = pyproj.Proj(crs)
        y,x = pyproj.transform(self.outProj,inp,latpos.longitude,latpos.latitude)
        XYZpos.x=y
        XYZpos.y=x
        XYZpos.z=0


    def cb(self,data):
        #print("RECEIVED GEO PATH MESSAGE")
        #print (data)
        if self.lastSatPos is None:
            print("geoPoseToPose: NO SAT DATA, FAIL")
            return
        #print (lastSatPos)
        ps = PoseStamped()
        ps.header = data.header
        ps.pose.orientation = data.pose.orientation
        self.transformCoordinates(data.pose.position,ps.pose.position)
        # move into map frame
        ps.header.frame_id='map'
        self.pub.publish(ps)
