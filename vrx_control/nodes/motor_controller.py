#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class Node():
    def __init__(self):
        self.pub = None
        self.left = 0
        self.right = 0
        self.cmd_vel = Twist()
        self.lin_kp = 2
        self.lin_ki = 1
        self.lin_kd = 0.1
        self.ang_kp = 2
        self.ang_ki = 5
        self.ang_kd = 0.2
        self.error_lin = 0
        self.error_ang = 0
        self.error_prior_lin = 0
        self.error_prior_ang = 0
        self.integral_ang = 0
        self.integral_lin = 0
        self.lin_vel = 0
        self.lin_vel_y = 0
        self.ang_vel = 0

        # Heartbeat Timer - if this time exceeds motors are cut off
        self.heartbeat_count = 0.
        self.heartbeat_cutoff_time = 1.0 # The max time exceeded before heartbeat

        self.left_pub = rospy.Publisher("left_cmd",Float32,queue_size=10)
        self.right_pub = rospy.Publisher("right_cmd",Float32,queue_size=10)
        self.left_msg = Float32()
        self.right_msg = Float32()

    def callback(self,data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))

        #self.driveMsg.left = data.linear.x
        #self.driveMsg.right = data.linear.x

        #self.driveMsg.left-=data.angular.z
        #self.driveMsg.right+=data.angular.z
        self.cmd_vel = data
        self.heartbeat_count = 0.


        #print("left: %f, right: %f"%(self.driveMsg.left,self.driveMsg.right))

    def callback_odom(self,data):
        twist_odom = data.twist.twist
        self.ang_vel = twist_odom.angular.z
        self.lin_vel = math.sqrt(twist_odom.linear.x**2 + twist_odom.linear.y**2) * twist_odom.linear.x/abs(twist_odom.linear.x)
        self.lin_vel_y = twist_odom.linear.y
        rospy.loginfo("Current speed is: x"+ str(twist_odom.linear.x)+"\t z:" +str(twist_odom.angular.z))


    def PID_loop(self,dt):
        k = 0.5

        self.heartbeat_count += dt
        if self.heartbeat_count < self.heartbeat_cutoff_time:
            self.error_prior_ang = self.error_ang
            self.error_prior = self.error_ang
            #ang = self.cmd_vel.angular.z -k * self.lin_vel_y*self.cmd_vel.linear.x

            self.error_lin =  self.cmd_vel.linear.x - self.lin_vel
            self.error_ang =  self.cmd_vel.angular.z - self.ang_vel
            self.integral_lin = self.integral_lin+(self.error_lin*dt)
            self.integral_ang = self.integral_ang+(self.error_ang*dt)
            derivative_lin = (self.error_lin - self.error_prior_lin)/dt
            derivative_ang = (self.error_ang - self.error_prior_ang)/dt


            self.left = self.lin_kp * self.error_lin + self.lin_ki * self.integral_lin + self.lin_kd * derivative_lin
            self.right = self.lin_kp * self.error_lin + self.lin_ki * self.integral_lin + self.lin_kd * derivative_lin

            self.left = self.left - (self.ang_kp * self.error_ang + self.ang_ki * self.integral_ang + self.ang_kd * derivative_ang)
            self.right = self.right + (self.ang_kp * self.error_ang + self.ang_ki * self.integral_ang + self.ang_kd * derivative_ang)
            #Publish the data

            self.left_msg.data = self.left
            self.right_msg.data = self.right

            self.left_pub.publish(self.left_msg)
            self.right_pub.publish(self.right_msg)

if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','cmd_vel')
    odom_topic = rospy.get_param('~odom_topic','odometry/filtered')

    node=Node()

    # Publisher

    # Subscriber
    rospy.Subscriber(in_topic,Twist,node.callback)
    rospy.loginfo("Subscribing to <%s>"%(odom_topic))

    rospy.Subscriber("odometry/filtered",Odometry,node.callback_odom)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            node.PID_loop(0.05)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
