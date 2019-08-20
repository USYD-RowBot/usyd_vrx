#!python
# I use venv on my computer so usr/bin/env python doesnt work for me
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
rospy.init_node("position_hold")

params = {
    "inTopic": "pointToHold",
    "outTopic": "waypoints"
}
rospy.get_param()
for p in rospy.get_params():
    params[p] = rospy.get_param(p, params[p])

waypointPub = rospy.Publisher(params["outTopic"], Path, queue_size=10)

# def cb(data):
#    global waypointPub
#    # publish a course.
#    waypointPub.publish(data)

#rospy.Subscriber(params["inTopic"],Twist,cb, queue_size=10)
rate = rospy.Rate(10)
tf_frame_id = "world"
while not rospy.is_shutdown():
    position = PoseStamped()
    path = Path()

    position.header.stamp = rospy.Time.now()
    position.header.frame_id = tf_frame_id

    path.header.stamp = rospy.Time.now()
    path.header.frame_id = tf_frame_id
    path.poses = [position]
    waypointPub.publish(path)
    rate.sleep()