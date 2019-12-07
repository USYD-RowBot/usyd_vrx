#!/usr/bin/env python
from vrx_msgs.srv import ClassifyPlacard, ClassifyPlacardResponse
import rospy
import rospkg
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError

def imageClient():
    rospy.wait_for_service('wamv/classify_placard')

    try:
        rospack = rospkg.RosPack()

        names = ["blue_circle",  "blue_cross",  "blue_triangle",
                "green_circle", "green_cross", "green_triangle",
                "red_circle",   "red_cross",   "red_triangle"]

        classifyPlacard = rospy.ServiceProxy('wamv/classify_placard', ClassifyPlacard)

        for name in names:
            cv2_img = cv2.imread(rospack.get_path('vrx_vision')+'/images/placards/' + name + '.png', cv2.IMREAD_COLOR)

            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(cv2_img, encoding="bgr8")

            res = classifyPlacard(img_msg)
            label = res.label
            print(label)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    imageClient()
