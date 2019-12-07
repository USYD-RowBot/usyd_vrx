#!/usr/bin/env python


from vrx_msgs.srv import ClassifyBuoy, ClassifyBuoyResponse
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from buoy_scanner import Scanner
import rospkg
import threading
import numpy as np
import time

# TODO:
# a counter to filter the error detection



class Cam():
    def __init__(self):
        # stream
        #t = threading.Thread(name='cam', target=self.capture)
        self.bridge = CvBridge()

        middle = rospy.Subscriber("wamv/sensors/cameras/middle_camera/image_raw",Image,self.capture)





    def capture(self,img):


        ros_img = img

        capturedImg = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")

        small_hsv = cv2.cvtColor(capturedImg, cv2.COLOR_BGR2HSV)

        # Threshold the image to remove unsaturated parts (want to keep the light)
        lower_colour = np.array([0,   150,   0]) # 0   0   100
        upper_colour = np.array([179, 255, 255]) # 179 255 255
        base_mask    = cv2.inRange(small_hsv, lower_colour, upper_colour)
        lower_colour = np.array([0,   0,    0])
        upper_colour = np.array([179, 100, 30])
        black_mask   = cv2.inRange(small_hsv, lower_colour, upper_colour)
        main_mask = cv2.bitwise_or(base_mask, black_mask)

        lower_colour = np.array([0, 0, 0]) # hue 92 - 148 is blue
        upper_colour = np.array([90, 255, 255])
        anti_blue1 = cv2.inRange(small_hsv, lower_colour, upper_colour)
        lower_colour = np.array([150, 0, 0])
        upper_colour = np.array([179, 255, 255])
        anti_blue2 = cv2.inRange(small_hsv, lower_colour, upper_colour)
        anti_blue = cv2.bitwise_or(anti_blue1, anti_blue2)

        combined_mask = cv2.bitwise_or(anti_blue, main_mask)
        inverted_img  = cv2.bitwise_not(combined_mask)

        cv2.imshow("image",capturedImg)

        cv2.imshow("mask",cv2.cvtColor(combined_mask,cv2.COLOR_GRAY2RGB))
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("Buoythresholding")

    c = Cam()
    rospy.spin()
