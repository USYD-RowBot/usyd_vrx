#!/usr/bin/env python

from vrx_msgs.srv import ClassifyBuoy, ClassifyBuoyResponse
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from buoy_scanner import Scanner
import rospkg
import threading

import time

# TODO:
# a counter to filter the error detection
class Cam(object):
    def __init__(self):
        # stream
        t = threading.Thread(name='cam', target=self.capture)
        t.start()
        self.bridge = CvBridge()


    def capture(self):
        while True:

            ros_img = rospy.wait_for_message("/wamv/sensors/cameras/middle_camera/image_raw", Image)

            self.capturedImg = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")


            #print(self.scanner.scanBuoy(self.capturedImg))
            time.sleep(0.1)

class SequenceFinder(object):
    def __init__(self):
        # init the scanner for buoy color detection
        self.scanner = Scanner()

        # some variables
        self.capturedImg = []
        self.result = []
        self.firstColor = ''
        self.prevColor = ''
        self.currentColor = ''
        self.idx = 0
        self.timeStep = 0.05
        self.sample = False
        self.counter = 0



    def colorFinder(self):

        if self.sample:
            # simulate the capture sequence
            a = ['red','red','none','none','blue','blue','blue','blue','green','green','green','green','red','red','red']
            # get the image
            # CHANGE THIS TO READ THE MIDDLE CAMERA FOR THE LIVE DETECTION
            rospack = rospkg.RosPack()
            img = cv2.imread(rospack.get_path('vrx_vision')+'/images/seq_'+a[self.idx]+'.png',cv2.IMREAD_COLOR)
            self.idx += 1
            # call the classify function
            # return the color of the buoy
            return self.scanner.scanBuoy(img)


        #ros_img = rospy.wait_for_message("/wamv/sensors/cameras/middle_camera/image_raw", Image)

        #self.capturedImg = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")

        #cv2.imshow('dd',self.capturedImg)
        #cv2.waitKey(1)

        return self.scanner.scanBuoy(cam.capturedImg)



    def run(self):
        self.currentColor = self.colorFinder()
        self.result.append(self.currentColor)
        self.firstColor = self.currentColor
        self.prevColor = self.currentColor

        while True:
            self.currentColor = self.colorFinder()
            #print(self.currentColor)
            #print(self.counter)
            #(self.result)
            cv2.imshow('dd',cam.capturedImg)
            cv2.waitKey(1)
            if self.currentColor != self.prevColor:
                if self.counter <3:
                    self.counter +=1
                    time.sleep(self.timeStep)
                    continue
                if self.currentColor == self.firstColor:
                    break
                #self.counter = 0
                self.result.append(self.currentColor)
                self.prevColor = self.currentColor
            else:
                self.counter = 0
            time.sleep(self.timeStep)

        if 'none' not in self.result:
            final = []
        else:
            final = self.result[self.result.index('none')+1:len(self.result)] + self.result[0:self.result.index('none')]
            if len(final) != 3:
                final = []
        #print('result')
        #print(self.result)
        return final



if __name__ == '__main__':

    cam = Cam()
    rospy.init_node("sequence_server")
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        sf = SequenceFinder()
        results = sf.run()

        #print('final')
        print(results)
        rate.sleep()
