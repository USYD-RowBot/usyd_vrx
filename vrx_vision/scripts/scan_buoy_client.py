#!/usr/bin/env python
from vrx_msgs.srv import ClassifyBuoy,ClassifyBuoyResponse
import rospy
import rospkg
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError

def imageClient():
    rospy.wait_for_service('wamv/scan_buoy')
    try:
        rospack = rospkg.RosPack()
        image = cv2.imread(rospack.get_path('vrx_vision')+'/images/seq_red.png',cv2.IMREAD_COLOR)
        cv2.imshow('img', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        scanBuoy = rospy.ServiceProxy('wamv/scan_buoy', ClassifyBuoy)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        res = scanBuoy(image_message, 0)
        print(res)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    imageClient()
