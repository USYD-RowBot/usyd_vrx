#!/usr/bin/env python

from vrx_msgs.srv import ClassifyBuoy, ClassifyBuoyResponse
import rospy
import rospkg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from placard_classifier import PlacardClassifier

def handleImage(req):

    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

    #print("Classifying placard")
    classifier        = PlacardClassifier()
    label, cX         = classifier.classifyPlacard(cv_img)

    return ClassifyBuoyResponse(label, cX, True)

def classifyPlacardServer():
    rospy.init_node('classify_placard')
    srv = rospy.Service('wamv/classify_placard', ClassifyBuoy, handleImage)
    rospy.spin()

if __name__ == "__main__":
    classifyPlacardServer()
