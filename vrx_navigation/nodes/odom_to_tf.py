#!/usr/bin/env python
import copy
import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

# Also create a fixed starting broadcast frame so rviz looks and works better (deprecated, just focus on base-link (but still use the witht0.rviz file cos it has a grid))

#t0frame=None

def handle_odom_pose(ros_odom_msg, tf_frame_id):
    #global t0frame
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t2 = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = tf_frame_id


    t.child_frame_id = "base_link"
    t.transform.translation.x = ros_odom_msg.pose.pose.position.x
    t.transform.translation.y = ros_odom_msg.pose.pose.position.y
    t.transform.translation.z = 0
    t.transform.rotation.x = ros_odom_msg.pose.pose.orientation.x
    t.transform.rotation.y = ros_odom_msg.pose.pose.orientation.y
    t.transform.rotation.z = ros_odom_msg.pose.pose.orientation.z
    t.transform.rotation.w = ros_odom_msg.pose.pose.orientation.w

    #Odom to map frame

    t2.child_frame_id = "odom"
    t2.header.frame_id="map"
    t2.transform.rotation.w = 1
    t2.header.stamp = rospy.Time.now()
    tf_prefix = rospy.search_param("tf_prefix")

    base_frame_id = "base_link"
    if tf_prefix is not None:
        base_frame_id = rospy.get_param(tf_prefix) + "/base_link"


    if base_frame_id != "base_link":
        t3 = geometry_msgs.msg.TransformStamped()
        t3.child_frame_id = base_frame_id
        t3.header.frame_id="base_link"
        t3.transform.rotation.w = 1
        t3.header.stamp = rospy.Time.now()
        br.sendTransform(t3)


    #if t0frame is None:
    #    t0frame = copy.deepcopy(t)
    #    t0frame.child_frame_id="t0home"
    #t0frame.header.stamp=rospy.Time.now()

    br.sendTransform(t)
    br.sendTransform(t2)
    #br.sendTransform(t0frame)
def callback(data):
    handle_odom_pose(data,"odom")


if __name__ == "__main__":
    rospy.init_node('odom_to_tf')
    rospy.Subscriber("odom",Odometry,callback)
    rospy.spin()
