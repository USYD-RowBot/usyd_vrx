#!/usr/bin/env python

from vrx_msgs.srv import ClassifyPlacard, ClassifyPlacardResponse
import rospy
import rospkg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from placard_classifier import PlacardClassifier

class ClassifyPlacardServer():

    def __init__(self):
        rospy.init_node('classify_placard')
        self.srv = rospy.Service('wamv/classify_placard', ClassifyPlacard, self.handleImage)

        self.classifier = PlacardClassifier()
        self.bridge = CvBridge()

    def handleImage(self, req):
        
        cv_img          = self.bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")
        label, conf, cX = self.classifier.classifyPlacard(cv_img)

        success = True
        if label == "":
            success = False

        return ClassifyPlacardResponse(label, conf, cX, success)
    
def main():
    srv = ClassifyPlacardServer()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.waitKey(1)
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()
