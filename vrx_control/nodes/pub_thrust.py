#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32



if __name__ == '__main__':

    rospy.init_node('pubthrust', anonymous=True)

    lpub = rospy.Publisher("/left_thrust_cmd",Float32,queue_size=1)
    rpub = rospy.Publisher("/right_thrust_cmd",Float32,queue_size=1)

    # thrust_cmd = rospy.get_param("thrust_cmd", 0.0)
    thrust_cmd = 150.0

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            thrust_msg = Float32()
            thrust_msg.data = thrust_cmd
            lpub.publish(thrust_cmd)
            rpub.publish(thrust_cmd)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
