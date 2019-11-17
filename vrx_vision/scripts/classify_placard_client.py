#!/usr/bin/env python
from vrx_msgs.srv import ClassifyBuoy,ClassifyBuoyResponse
import rospy
import rospkg
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from placard_classifier import PlacardClassifier

def imageClient():
    rospy.wait_for_service('wamv/classify_placard')

    try:
        rospack = rospkg.RosPack()
        classifier = PlacardClassifier()

        names = ["blue_circle",  "blue_cross",  "blue_triangle",
                "green_circle", "green_cross", "green_triangle",
                "red_circle",   "red_cross",   "red_triangle"]

        for name in names:
            img = cv2.imread(rospack.get_path('vrx_vision')+'/images/placards/' + name + '.png',cv2.IMREAD_COLOR)
            label, confidence = classifier.classifyPlacard(img)

            classifyPlacard = rospy.ServiceProxy('wamv/classify_placard', ClassifyBuoy)

            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")

            res = classifyPlacard(img_msg, 0)
            print(res)

            cv2.imshow('img', img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    imageClient()
