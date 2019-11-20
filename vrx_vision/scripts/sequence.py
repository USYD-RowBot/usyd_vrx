#!/usr/bin/env python
from vrx_msgs.srv import ClassifyBuoy, ClassifyBuoyResponse
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from buoy_scanner import Scanner
import rospkg

import time

# TODO:
# a counter to filter the error detection


class SequenceFinder(object):
    def __init__(self):
        # init the scanner for buoy color detection
        self.scanner = Scanner()

        # some variables
        self.result = []
        self.firstColor = ''
        self.prevColor = ''
        self.currentColor = ''
        self.idx = 0
        self.timeStep = 0.01

    def colorFinder(self):

        if True:
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


    def run(self):
        self.currentColor = self.colorFinder()
        self.result.append(self.currentColor)
        self.firstColor = self.currentColor
        self.prevColor = self.currentColor

        while True:
            self.currentColor = self.colorFinder()
            if self.currentColor != self.prevColor:
                if self.currentColor == self.firstColor:
                    break

                self.result.append(self.currentColor)
                self.prevColor = self.currentColor
            time.sleep(self.timeStep)

        final = self.result[self.result.index('none')+1:len(self.result)] + self.result[0:self.result.index('none')]
        return final


if __name__ == '__main__':
    sf = SequenceFinder()
    results = sf.run()
    print(results)
