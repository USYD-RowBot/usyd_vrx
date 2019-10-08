#!/usr/bin/env python
import copy
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def handle_odom_pose(ros_odom_msg):

    br = tf2_ros.TransformBroadcaster()

    map_2_odom                 = geometry_msgs.msg.TransformStamped()
    odom_2_wamv_odom           = geometry_msgs.msg.TransformStamped()
    wamv_base_link_2_base_link = geometry_msgs.msg.TransformStamped()
    
    map_2_odom.header.stamp = rospy.Time.now()
    map_2_odom.header.frame_id = "/map"
    map_2_odom.child_frame_id = "/odom"
    map_2_odom.transform.rotation.w = 1

    odom_2_wamv_odom.header.stamp = rospy.Time.now()
    odom_2_wamv_odom.header.frame_id = "/odom"
    odom_2_wamv_odom.child_frame_id = "/wamv/odom"
    odom_2_wamv_odom.transform.rotation.w = 1

    wamv_base_link_2_base_link.header.stamp = rospy.Time.now()
    wamv_base_link_2_base_link.header.frame_id = "/wamv/base_link"
    wamv_base_link_2_base_link.child_frame_id = "/base_link"
    wamv_base_link_2_base_link.transform.rotation.w = 1

    br.sendTransform(map_2_odom)
    br.sendTransform(odom_2_wamv_odom)
    br.sendTransform(wamv_base_link_2_base_link)
    
def callback(data):
    handle_odom_pose(data)

if __name__ == "__main__":
    rospy.init_node('wamv_tf_expander')
    rospy.Subscriber("odom",Odometry,callback)
    rospy.spin()
