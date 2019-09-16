#!/usr/bin/env python

from vrx_msgs.srv import ClassifyBuoy,ClassifyBuoyResponse
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def handleImage(req):
    #print(req)

    bridge = CvBridge()

    distance = req.distance
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
    ##cv_image is now the image you can work with.
    cv2.imshow("service reciever",cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #Return with your results
    return ClassifyBuoyResponse("Hello",1.1,True)

def classifyBuoyServer():
    rospy.init_node('classify_bioy')
    s = rospy.Service('classify_buoy', ClassifyBuoy, handleImage)
    print "Ready classify buoy."
    rospy.spin()

if __name__ == "__main__":
    classifyBuoyServer()
