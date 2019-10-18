#!/usr/bin/env python

from vrx_msgs.srv import ClassifyBuoy, ClassifyBuoyResponse
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from buoy_scanner import Scanner

def handleImage(req):

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

    print("Scanning buoy")
    scanner = Scanner()
    scanned_colour = scanner.scanBuoy(cv_image)

    #cv2.imshow("service receiver", cv_image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    #Return with your results
    print(scanned_colour)
    return ClassifyBuoyResponse(scanned_colour, 1.0, True)

def scanBuoyServer():
    rospy.init_node('scan_buoy')
    s = rospy.Service('wamv/scan_buoy', ClassifyBuoy, handleImage)
    print("Ready to scan buoys.")
    rospy.spin()

if __name__ == "__main__":
    scanBuoyServer()
