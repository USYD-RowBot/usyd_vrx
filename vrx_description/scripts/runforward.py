#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
rospy.init_node("runforward")
pub_left= rospy.Publisher("right_thrust_cmd",Float32,queue_size=10)
pub_right= rospy.Publisher("left_thrust_cmd",Float32,queue_size=10)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub_left.publish(1)
    pub_right.publish(1)
    rate.sleep()