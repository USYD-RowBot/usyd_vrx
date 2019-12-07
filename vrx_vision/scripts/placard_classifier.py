#!/usr/bin/env python
import cv2
import numpy as np
import rospkg
from classifier import Classifier
import rospy

TRACKBAR = False

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
        if TRACKBAR:
            self.setupSliders()

    def setupSliders(self):
        cv2.namedWindow('Sliders') # Create a window for the sliders

        # Create sliders for HSV thresholding
        cv2.createTrackbar('H_lower', 'Sliders', 0, 180, self.nothing)
        cv2.createTrackbar('H_upper', 'Sliders', 180, 180, self.nothing)

        cv2.createTrackbar('S_lower', 'Sliders', 220, 255, self.nothing)
        cv2.createTrackbar('S_upper', 'Sliders', 255, 255, self.nothing)

        cv2.createTrackbar('V_lower', 'Sliders', 0, 255, self.nothing)
        cv2.createTrackbar('V_upper', 'Sliders', 130, 255, self.nothing)

        cv2.createTrackbar('Blackout Bottom %', 'Sliders', 50, 100, self.nothing)
        cv2.createTrackbar('Max Contours', 'Sliders', 10, 30, self.nothing)


    def getImageMask(self, img):
        # Convert to HSV colour space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if TRACKBAR:
            H_lower = cv2.getTrackbarPos('H_lower', 'Sliders')
            S_lower = cv2.getTrackbarPos('S_lower', 'Sliders')
            V_lower = cv2.getTrackbarPos('V_lower', 'Sliders')

            H_upper = cv2.getTrackbarPos('H_upper', 'Sliders')
            S_upper = cv2.getTrackbarPos('S_upper', 'Sliders')
            V_upper = cv2.getTrackbarPos('V_upper', 'Sliders')

            blackout_pcnt = cv2.getTrackbarPos('Blackout Bottom %', 'Sliders')
            lower_colour = np.array([H_lower, S_lower, V_lower]) # [  0, 220,   0]
            upper_colour = np.array([H_upper, S_upper, V_upper]) # [180, 255, 130]


        else:

            # Threshold the image to remove unsaturated parts
            lower_colour = np.array([0, 220, 0]) # [  0, 220,   0]
            upper_colour = np.array([180, 255, 255]) # [180, 255, 130]
            blackout_pcnt = 50
        mask = cv2.inRange(hsv, lower_colour, upper_colour)

        # Blur and threshold to get rid of some noise
        blur = cv2.GaussianBlur(src=mask, ksize=(5, 5), sigmaX=0)
        (_, binary) = cv2.threshold(src=blur,
            thresh=60,
            maxval=255,
            type=cv2.THRESH_BINARY)

        rows = binary.shape[0] # Work out row indices between which to blackout
        blackout_row = int(float(100 - blackout_pcnt)*float(rows)/100)

        binary[blackout_row:rows] = 0 # Blackout the bottom

        return binary

    def getSymbolContours(self, img):
        # Crop image to buoy
        cont_return = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

        if len(contours) == 0:
            print("Couldn't find contours for image mask. Mask might not include a placard symbol.")
            return None

        #return max(contours, key=cv2.contourArea) # Get largest contour
        return contours

    def getCentreColour(self, img, cnt):
        cX, cY = self.getCentre(img, cnt)
        return self.bgr2rgb(img[cY, cX, :])

    def getCentre(self, img, cnt):
        M = cv2.moments(cnt)
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except ZeroDivisionError:
            cY = cnt[0, 0, 0] # Select first point in contour
            cX = cnt[0, 0, 1]
        return cX, cY

    def cropScale(self, img, cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        cropped_img = img[y:y+h, x:x+w] # Crop rectangle around symbol

        width = 100  # Scale image to constant size
        scaled_img = cv2.resize(cropped_img, (width, width))
        return scaled_img

    def nothing(self, x):
        pass

    def classifyPlacard(self, img):

        scale_factor = 0.5
        small_img    = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)

        if TRACKBAR:
            cv2.imshow("small_img", small_img)
            cv2.waitKey(1)

        mask = self.getImageMask(small_img)

        contours = self.getSymbolContours(mask)
        if contours is not None:
            contours.sort(key=cv2.contourArea, reverse=True)
            if TRACKBAR:
                max_contours = cv2.getTrackbarPos('Max Contours', 'Sliders') # Limit number of contours
            else:
                max_contours = 10
            if len(contours) > max_contours:
                contours = contours[0:max_contours]

            best_label      = ""
            best_confidence = 0

            for cnt in contours:
                centre_colour = self.getCentreColour(small_img, cnt)
                scaled_img    = self.cropScale(mask, cnt)

                label, conf_shape, conf_colour = self.classifyImage(scaled_img, centre_colour)

                if conf_shape*conf_colour > best_confidence:
                    best_label      = label
                    best_confidence = conf_shape*conf_colour

            rospy.logdebug("Label: %s\nConfidence: %s\n" % (best_label, best_confidence))

            if TRACKBAR:
                cv2.imshow("mask", mask)
                cv2.waitKey(1)

                cv2.destroyWindow("small_img")
                cv2.destroyWindow("mask")

            # Return x value of centre of symbol instead of confidence, hack for docking action server
            cX, _ = self.getCentre(small_img, cnt)
            cX    = int(cX/scale_factor)

            return best_label, best_confidence, cX
        else:
            return "", 0.0, 640
