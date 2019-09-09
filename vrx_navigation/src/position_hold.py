#!/usr/bin/env python
# I use venv on my computer so usr/bin/env python doesnt work for me
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
rospy.init_node("position_hold")

params = {
    "inTopic": "pointToHold",
    "outTopic": "/waypoints"
}
for i in params:
    params[i] = rospy.get_param('~'+i, params[i])

waypointPub = rospy.Publisher(params["outTopic"], Path, queue_size=10)

# def cb(data):
#    global waypointPub
#    # publish a course.
#    waypointPub.publish(data)

#rospy.Subscriber(params["inTopic"],Twist,cb, queue_size=10)
rate = rospy.Rate(1)
tf_frame_id = "map"


# subscribe to odometry to get initial position for point holding when this is run
listener = tf.TransformListener()
'odom'

tf0 = None

while not rospy.is_shutdown():
    global tf0
    if (tf0 is None):
        try:
            tf0 = listener.lookupTransform(
                '/base_link', tf_frame_id, rospy.Time(0))
            print("lookup ok")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    if not (tf0 is None):
        print("publish hold")
        print (tf0)
        position = PoseStamped()
        path = Path()
        position.pose.position.x=tf0[0][0]
        position.pose.position.y=tf0[0][1]
        position.pose.position.z=tf0[0][2]
        position.pose.orientation.x=tf0[1][0]
        position.pose.orientation.y=tf0[1][1]
        position.pose.orientation.z=tf0[1][2]
        position.pose.orientation.w=tf0[1][3]
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = tf_frame_id

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = tf_frame_id
        path.poses = [position]
        waypointPub.publish(path)
        try:
            tf1 = listener.lookupTransform(
                '/base_link', tf_frame_id, rospy.Time(0))
            print("current pos:")
            print(tf1)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    rate.sleep()
