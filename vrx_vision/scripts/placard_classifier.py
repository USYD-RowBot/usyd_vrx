#!/usr/bin/env python
import cv2
import numpy as np
import rospkg

class PlacardClassifier():

    def __init__(self):
        rospack = rospkg.RosPack()
        pre = rospack.get_path('vrx_vision')+"/template_images/template_placard_"
        filetype = ".png"
        self.template_filename_list = []

        self.template_labels = ["circle", "cross", "triangle"]

        self.template_colours = [ # RGB colours
            [(0, 0, 120)], # Blue
            [(0, 120, 0)]  # Green
            [(120, 0, 0)]] # Red

        for label in self.template_labels:
            self.template_filename_list.append(pre + label + filetype)

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

    def cropScale(self, img):
        # Crop image to buoy
        cont_return = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

        if len(contours) == 0:
            print("Couldn't find contours for image mask. Mask might not include a placard symbol.")
            return None

        best_cnt = max(contours, key=cv2.contourArea) # Get largest contour
        x, y, w, h = cv2.boundingRect(best_cnt)
        cropped_img = img[y:y+h, x:x+w] # Crop rectangle around buoy

        # Scale image to constant size
        width = 100
        scaled_img = cv2.resize(cropped_img, (width, width))
        return scaled_img

    def classifyPlacard(self, img):
        
        scale_factor = 0.5
        small_img = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)
        mask = self.getImageMask(small_img)
        scaled = self.cropScale(mask)

        cv2.imshow("scaled", scaled)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        