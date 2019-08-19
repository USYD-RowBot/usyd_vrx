#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import euler_from_quaternion

rospy.init_node('goal_to_path')
path_pub = rospy.Publisher("waypoints", Path, queue_size=1)



def goalCallback(data):
    path = Path()
    path.header.frame_id = "map"
    poses = []
    poses.append(data)
    path.poses = poses
    path_pub.publish(path)


if __name__ == "__main__":

    rospy.Subscriber("move_base_simple/goal", PoseStamped, goalCallback)

    rospy.spin()
