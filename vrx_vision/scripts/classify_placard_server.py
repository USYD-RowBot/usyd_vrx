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
    cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding="bgr8")

    print("Classifying placard")
    classifier = PlacardClassifier()
    label = classifier.classifyPlacard(cv_image)

    #cv2.imshow("service receiver", cv_image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    #Return with your results
    print(label)
    return ClassifyBuoyResponse(label, 1.0, True)

def scanBuoyServer():
    rospy.init_node('classify_placard')
    #s = rospy.Service('wamv/classify_placard', ClassifyBuoy, handleImage)
    test()
    #rospy.spin()

def test():
    rospack = rospkg.RosPack()

    names = ["blue_circle",  "blue_cross",  "blue_triangle",
            "green_circle", "green_cross", "green_triangle",
              "red_circle",   "red_cross",   "red_triangle"]

    for name in names:
        img = cv2.imread(rospack.get_path('vrx_vision')+'/images/placards/' + name + '.png',cv2.IMREAD_COLOR)

        #cv2.imshow('img', img)
        #cv2.waitKey(0)

        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")

        classifier = PlacardClassifier()
        label = classifier.classifyPlacard(img)


if __name__ == "__main__":
    scanBuoyServer()
