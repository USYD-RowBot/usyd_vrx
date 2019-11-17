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

class geoPathToPathConverter:
    def __init__(self):
        params = {
            "inTopic": "/pointToHold",
            "outTopic": "/waypoints",
            "gpsTopic": "wamv/sensors/gps/gps/fix",
        }
        self.outProj = pyproj.Proj("+init=EPSG:4326")
        #rospy.init_node("geoPathToPath",anonymous=True)
        for i in params:
            params[i] = rospy.get_param('~'+i, params[i])

        self.pub = rospy.Publisher(params['outTopic'], Path, queue_size=1)
        self.lastSatPos=None
        self.listener = tf.TransformListener()
        rospy.Subscriber(params['gpsTopic'], NavSatFix, self.satcb)
        rospy.Subscriber(params['inTopic'], GeoPath, self.cb)

    def satcb(self,data):
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
            print("NO SAT DATA, FAIL")
            return
        #print (lastSatPos)
        path = Path()
        path.header = data.header
        path.header.frame_id="map"# not sure why this isnt published :3
        poses = []
        for dps in data.poses:
            ps = PoseStamped()
            ps.header = dps.header
            ps.pose.orientation = dps.pose.orientation
            self.transformCoordinates(dps.pose.position,ps.pose.position)
            # move into map frame
            ps.header.frame_id='map'
            poses.append(ps)
        path.poses=poses
        self.pub.publish(path)
        #print(path)


