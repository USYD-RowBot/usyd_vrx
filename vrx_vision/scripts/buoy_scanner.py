#!/usr/bin/env python
import cv2
import numpy as np




class Scanner():

  def detectLightBuoy(self, c):
	# initialize the shape name and approximate the contour
    isLightBuoy = False
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    if len(approx) == 4:
	# compute the bounding box of the contour and use the
	# bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)

		# a square will have an aspect ratio that is approximately
		# equal to one, otherwise, the shape is a rectangle
        if ar<0.9:
            isLightBuoy = True
	# return the name of the shape
    return isLightBuoy

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

    xx,yy =binary.shape
    mask = mask[int(xx/3):int(2*xx/3),:]
    img = img[int(xx/3):int(2*xx/3),:,:]
    cv2.imshow("binary", mask)
    cv2.waitKey(1)

    # Find contours of image
    cont_return = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

    # Find contours with four sides, rectangles
    four_cnts = []
    for cnt in contours:
        if self.detectLightBuoy(cnt):
            four_cnts.append(cnt)
    # Get the smallest 4-contour
    if len(four_cnts) == 0:
        #print('nocnts')
        return "none"
    best_cnt = max(four_cnts, key=cv2.contourArea)
    #print(best_cnt)
    # Calculate aspect ratio to check that it is actually the light
    x,y,w,h = cv2.boundingRect(best_cnt)
    aspect_ratio = float(w)/h

    if aspect_ratio < 0.75:
      # Get the center of the contour
      M = cv2.moments(best_cnt)
      try:
          cX = int(M["m10"] / M["m00"])
          cY = int(M["m01"] / M["m00"])
      except:
          print('divided by 0')
          return "none"
      center_pixel = img[cY, cX, :]
      #print(center_pixel)
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
