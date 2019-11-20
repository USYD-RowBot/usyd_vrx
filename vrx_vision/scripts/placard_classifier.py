#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
from classifier import Classifier

class PlacardClassifier(Classifier):

    def __init__(self):
        super(PlacardClassifier, self).__init__() # Inherit methods from Classifier parent

        self.template_labels = [
            ["blue_circle",     "green_circle",   "red_circle"], 
            ["blue_cross",       "green_cross",    "red_cross"], 
            ["blue_triangle", "green_triangle", "red_triangle"]]

        self.template_colours = [ # RGB colours
            [(0, 0, 120), (0, 120, 0), (120, 0, 0)], # blue, green, red
            [(0, 0, 120), (0, 120, 0), (120, 0, 0)],
            [(0, 0, 120), (0, 120, 0), (120, 0, 0)]]

        pre = rospkg.RosPack().get_path('vrx_vision')+"/template_images/template_placard_"

        self.template_filename_list = [
            pre + "circle.png",
            pre + "cross.png",
            pre + "triangle.png"
        ]

    def getImageMask(self, img):
        # Convert to HSV colour space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

        # Threshold the image to remove unsaturated parts
        lower_colour = np.array([0,   220,  0])
        upper_colour = np.array([180, 255, 130])
        mask = cv2.inRange(hsv, lower_colour, upper_colour) 

        # Blur and threshold to get rid of some noise
        blur = cv2.GaussianBlur(src=mask, ksize=(5, 5), sigmaX=0)
        (_, binary) = cv2.threshold(src=blur,
            thresh=60, 
            maxval=255, 
            type=cv2.THRESH_BINARY)

        return binary

    def getSymbolContour(self, img, original_img):
        # Crop image to buoy
        cont_return = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

        if len(contours) == 0:
            print("Couldn't find contours for image mask. Mask might not include a placard symbol.")
            cv2.imshow("mask", img)
            cv2.imshow("original_img", original_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            return None

        return max(contours, key=cv2.contourArea) # Get largest contour

    def getCentreColour(self, img, cnt):
        M = cv2.moments(cnt)          
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return self.bgr2rgb(img[cY, cX, :])

    def getCentre(self, img, cnt):
        M = cv2.moments(cnt)          
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY

    def cropScale(self, img, cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        cropped_img = img[y:y+h, x:x+w] # Crop rectangle around symbol

        width = 100  # Scale image to constant size
        scaled_img = cv2.resize(cropped_img, (width, width))
        return scaled_img

    def classifyPlacard(self, img):
        
        scale_factor = 0.5
        small_img    = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)

        mask = self.getImageMask(small_img)
        cnt  = self.getSymbolContour(mask, small_img)

        if cnt is not None:
            centre_colour = self.getCentreColour(small_img, cnt)
            scaled_img    = self.cropScale(mask, cnt)
            
            label, conf_shape, conf_colour = self.classifyImage(scaled_img, centre_colour)
            #print("Label: %s\nShape Confidence: %s\nColour Confidence: %s\n" % (label, conf_shape, conf_colour))

            cX, _ = self.getCentre(small_img, cnt)
            cX    = int(cX/scale_factor)

            return label, cX
        else:
            return "", 640

        