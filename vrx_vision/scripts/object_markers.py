#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from vrx_msgs.msg import ObjectArray, Object


def callback(data):
    pub = rospy.Publisher("visualization_marker",Marker,queue_size = 10)
    #print(data)
    objects = data.objects
    for i in objects:

        if len(i.confidences) == 0:
            pass

        if i.best_guess == "buoy2":

            marker = Marker()
            marker.header.frame_id = i.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic text"
            marker.id = int(i.frame_id)
            marker.type = Marker.MESH_RESOURCE
            marker.action=Marker.ADD
            marker.mesh_resource="package://vrx_vision/mesh/buoy.dae"
            #marker.text = i.best_guess +" " +  i.frame_id
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(5)
            pub.publish(marker)
        if i.best_guess == "dock":

            marker = Marker()
            marker.header.frame_id = i.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic text"
            marker.id = int(i.frame_id)
            marker.type = Marker.MESH_RESOURCE
            marker.action=Marker.ADD
            marker.mesh_resource="package://vrx_vision/mesh/dock2.dae"
            #marker.text = i.best_guess +" " +  i.frame_id
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 0.2
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(5)
            pub.publish(marker)
        else:
            marker = Marker()
            marker.header.frame_id = i.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic text"
            marker.id = int(i.frame_id)+1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action=Marker.ADD
            marker.text = "ID: " +i.frame_id +"\nTYPE: " + i.best_guess +"\nCONF: " +  str(i.confidences[0])
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(5)
            pub.publish(marker)
if __name__ == "__main__":
    rospy.init_node("simple_marker")
    rospy.Subscriber("objects", ObjectArray, callback)
    rospy.spin()
