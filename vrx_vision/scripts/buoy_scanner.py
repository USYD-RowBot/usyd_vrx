#!/usr/bin/env python
import cv2
import numpy as np

class Scanner():

  def classifyColour(self, pixel):
    '''
    Checks which colour the buoy is flashing.
    '''
    colour_vals = [[121, 0,  0], [0, 119, 0], [0, 0, 118]]
    colour_names = ["blue", "green", "red"]
    colour_dists = []

    for colour in colour_vals:
      R_dist = colour[0] - pixel[0]
      G_dist = colour[1] - pixel[1]
      B_dist = colour[2] - pixel[2]
      colour_dists.append(np.sqrt(R_dist**2 + G_dist**2 + B_dist**2))

    return colour_names[colour_dists.index(min(colour_dists))]

  def scanBuoy(self, img):
    ''' 
    Scans an image of the light sequence buoy to get the colour.

    Return (string): "red", "green, "blue", "none"
    '''
    #filename = "scan_imgs/green.png"
    #cv_image = cv2.imread(filename)

    # Change to HSV colour space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

    # Threshold the image to remove unsaturated parts (want to keep the light)
    lower_colour = np.array([0, 140, 60])
    upper_colour = np.array([360, 360, 180])
    mask = cv2.inRange(hsv, lower_colour, upper_colour) 

    # Blur and threshold to get rid of some noise
    blur = cv2.GaussianBlur(src=mask, ksize=(3, 3), sigmaX=0)
    (_, binary) = cv2.threshold(src=blur,
        thresh=60, 
        maxval=255, 
        type=cv2.THRESH_BINARY)

    #cv2.imshow("binary", binary)
    #cv2.waitKey(0)

    # Find contours of image
    cont_return = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

    # Find contours with four sides, rectangles
    four_cnts = []
    for cnt in contours:
      if len(cnt) is 4:
        four_cnts.append(cnt)

    # Get the smallest 4-contour
    best_cnt = max(four_cnts, key=cv2.contourArea)

    # Calculate aspect ratio to check that it is actually the light
    x,y,w,h = cv2.boundingRect(best_cnt)
    aspect_ratio = float(w)/h

    if aspect_ratio < 0.75:
      # Get the center of the contour
      M = cv2.moments(best_cnt)
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      center_pixel = img[cY, cX, :]

      return self.classifyColour(center_pixel)
    else:
      return "none"

    '''cv2.drawContours(image = img, 
        contours = [best_cnt], 
        contourIdx = -1, 
        color = (0, 255, 0), 
        thickness = 1)
    cv2.imshow("contours", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()'''