#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge() # This is a standard ROS package for converting ros image messages to CV images.
def callback(data):
    global bridge
    image=bridge.imgmsg_to_cv2(data, "bgr8")
    # Work with the image here - imshow, etc





if __name__ == "__main__":
    rospy.init_node("image_topic_shell")
    rospy.Subscriber("front_camera/image_raw", Image, callback)
    rospy.spin()