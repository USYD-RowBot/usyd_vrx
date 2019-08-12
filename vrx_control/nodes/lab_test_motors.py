#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import random


if __name__ == '__main__':

    rospy.init_node('labtestmotors', anonymous=True)

    command_limit = 200.
    time_period_limit = 10.

    lpub = rospy.Publisher("/left_thrust_cmd",Float32,queue_size=1)
    rpub = rospy.Publisher("/right_thrust_cmd",Float32,queue_size=1)

    thrust_on = True
    publish_thrust = True
    time_period = random.uniform(0, time_period_limit)
    time_last = rospy.get_rostime().to_sec()

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            if publish_thrust:
                if thrust_on:
                    thrust_msg = Float32()
                    thrust_cmd = random.uniform(0, command_limit)
                    thrust_msg.data = thrust_cmd
                    lpub.publish(thrust_cmd)
                    rpub.publish(thrust_cmd)
                    print("Left %f, Right %f" %(thrust_cmd, thrust_cmd))
                else:
                    thrust_msg = Float32()
                    thrust_msg.data = 0.0
                    lpub.publish(thrust_msg)
                    rpub.publish(thrust_msg)
                    print("Left %f, Right %f" %(0.0, 0.0))
            else:
                print("Not publishing")
            rate.sleep()
            if rospy.get_rostime().to_sec() > time_last + time_period:
                time_last = rospy.get_rostime().to_sec()
                time_period = random.uniform(0, time_period_limit)
                thrust_on = bool(random.randint(0, 2))
                publish_thrust = bool(random.randint(0, 2))
    except rospy.ROSInterruptException:
        pass
