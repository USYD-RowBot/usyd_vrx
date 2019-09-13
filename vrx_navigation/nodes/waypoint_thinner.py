#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
import math
from tf.transformations import euler_from_quaternion

class WaypointThinner:
    def __init__(self):
        self.pub_ = rospy.Publisher("waypoints", Path, queue_size=1)
        rospy.Subscriber("/waypoints_full", Path, self.pathCallback)

    def pathCallback(self,path_msg):
        thinned_msg = Path()
        thinned = []
        last_pose = Pose()
        count = 0
        count2 = 0;
        for pose_stamped in path_msg.poses:
            pose = pose_stamped.pose





            if count%4 == 0:
                pose_stamped.header.seq = count2
                count2 = count2+1
                thinned.append(pose_stamped)
                last_pose = pose

            count = count+1

        thinned_msg.poses = thinned
        thinned_msg.header.stamp = rospy.Time.now()
        thinned_msg.header.frame_id = "map"
        self.pub_.publish(thinned_msg)




if __name__ == "__main__":
    rospy.init_node('paththinner')
    wt = WaypointThinner()
    rospy.spin()
