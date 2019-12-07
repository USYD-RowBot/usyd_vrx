import numpy as np
import cv2
import rospkg
import rospy

import sys
rospack = rospkg.RosPack()
sys.path.insert(1, rospack.get_path('vrx_vision')+"/scripts")
from classifier import Classifier

class BuoyClassifier(Classifier):

    def __init__(self, exclusion_list, hfov, cam_x_px):
        '''
            exclusion list: list of strings detailing which buoys should be removed from search list
            hfov: camera horizontal field of view
            cam_x_px: x resolution of camera
        '''
        super(BuoyClassifier, self).__init__() # Inherit methods from Classifier parent

        ##### CAMERA PARAMS #####
        self.focal_length = (float(cam_x_px)/2)/np.tan(float(hfov)/2)

        ##### IMAGE TEMPLATE STUFF #####
        rospack = rospkg.RosPack()
        pre = rospack.get_path('vrx_vision')+"/template_images/"

        self.template_filename_list = [
            pre+"template_conical.png",
            pre+"template_tophat.png",
            pre+"template_totem.png",
            pre+"template_sphere.png",
            pre+"template_scan.png"]

        self.template_colours = [ # RGB colours
            [(169, 71, 65)],                                                   # Conical
            [(255, 255, 255), (118, 203, 166)],                                # Tophat: (white, green)
            [(255, 255, 0), (1, 1, 1), (4, 4, 255), (4, 255, 4), (255, 4, 4)], # Totem:  (yellow, black, blue, green, red)
            [(1, 1, 1)],                                                       # Sphere
            [(20, 20, 20)]                                                     # Scan buoy
        ]

        self.template_labels = [
            ["surmark950410"],                                                        # Conical
            ["surmark46104", "surmark950400"],                                       # Tophat: (white, green)
            ["yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem"], # Totems
            ["polyform"],                                                              # Sphere TODO estimate size
            ["scan_buoy"]                                                              # Scan buoy
        ]

        ##### REMOVE EXCLUDED BUOYS #####
        template_list_length = len(self.template_labels)
        remove_i = []

        for i in range(template_list_length):
            category_length = len(self.template_labels[i])
            remove_j = []

            for j in range(category_length):
                for excluded_buoy in exclusion_list:
                    if self.template_labels[i][j] == excluded_buoy:
                        remove_j.append(j) # Mark buoy for deletion

            if len(remove_j) == category_length:
                remove_i.append(i) # Mark category for deletion
            else:
                for k in range(len(remove_j)-1, -1, -1):
                    del self.template_labels[i][remove_j[k]] # Delete particular buoy
                    del self.template_colours[i][remove_j[k]]

        for k in range(len(remove_i)-1, -1, -1): # Delete whole category
            del self.template_labels[remove_i[k]]
            del self.template_colours[remove_i[k]]
            del self.template_filename_list[remove_i[k]]

        #### CLASS VARS #####
        self.centre_colour = None

        #print(self.template_labels)
        #print(self.template_colours)
        #print(self.template_filename_list)

    def kMeans(self, img):

        #img = cv2.resize(img, (500, 500))
        img = cv2.GaussianBlur(img, (5, 5), cv2.BORDER_CONSTANT)

        Z = img.reshape((-1,3))
        Z = np.float32(Z) # convert image to numpy float32

        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 10
        _,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

        # Now convert back into uint8, and make original image
        center = np.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))

        #cv2.imshow('K-Means Clustering', res2)
        #cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return res2

    def centreColour(self, img):

        #Resize image
        scale_factor = 0.5
        small_img    = cv2.resize(img, (0,0), fx=scale_factor, fy=scale_factor)

        # Change to HSV colour space
        small_hsv = cv2.cvtColor(small_img, cv2.COLOR_BGR2HSV)

        # Threshold the image to remove unsaturated parts (want to keep the light)
        lower_colour = np.array([0,   150,   0]) # 0   0   100
        upper_colour = np.array([179, 255, 255]) # 179 255 255
        base_mask    = cv2.inRange(small_hsv, lower_colour, upper_colour)
        lower_colour = np.array([0,   0,    0])
        upper_colour = np.array([179, 100, 30])
        black_mask   = cv2.inRange(small_hsv, lower_colour, upper_colour)
        main_mask = cv2.bitwise_or(base_mask, black_mask)

        lower_colour = np.array([0, 0, 0]) # hue 92 - 148 is blue
        upper_colour = np.array([90, 255, 255])
        anti_blue1 = cv2.inRange(small_hsv, lower_colour, upper_colour)
        lower_colour = np.array([150, 0, 0])
        upper_colour = np.array([179, 255, 255])
        anti_blue2 = cv2.inRange(small_hsv, lower_colour, upper_colour)
        anti_blue = cv2.bitwise_or(anti_blue1, anti_blue2)

        combined_mask = cv2.bitwise_or(anti_blue, main_mask)
        inverted_img  = cv2.bitwise_not(combined_mask)

        # apply a gaussian blur over the image to create a more bulbous shape for blob detection
        #blurred_img = cv2.blur(mask,(64, 16))
        #_,rounded_img = cv2.threshold(blurred_img,254,255,cv2.THRESH_BINARY)

        border_w = 2
        bordered_img = cv2.copyMakeBorder(inverted_img, border_w, border_w, border_w, border_w, # Add white border
        cv2.BORDER_CONSTANT, value=(255, 255, 255))

        params = cv2.SimpleBlobDetector_Params()

        #params.minThreshold = 10
        #params.maxThreshold = 200

        params.filterByArea = True
        params.minArea = 20

        #params.filterByCircularity = True
        #params.minCircularity = 0.1

        params.filterByConvexity = False
        #params.minConvexity = 0.1

        params.filterByInertia = False
        #params.minInertiaRatio = 0.3

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(bordered_img)
        im_with_keypoints = cv2.drawKeypoints(bordered_img, keypoints,
                    np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        #cv2.imshow("Keypoints", im_with_keypoints)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        debug = combined_mask
        if len(keypoints) == 0: # No blob found, assume this is ocean
            '''lower_colour = np.array([0, 127, 0]) # hue 92 - 148 is blue
            upper_colour = np.array([92, 255, 255])
            anti_blue1 = cv2.inRange(hsv, lower_colour, upper_colour)
            lower_colour = np.array([148, 127, 0])
            upper_colour = np.array([179, 255, 255])
            anti_blue2 = cv2.inRange(hsv, lower_colour, upper_colour)
            anti_blue = cv2.bitwise_or(anti_blue1, anti_blue2)'''
            return None,debug

        x = keypoints[0].pt[0] # Get centre coords
        y = keypoints[0].pt[1]

        #scale_factor_inv = 1.0/scale_factor
        #centre = img[int(np.floor(scale_factor_inv*(y-border_w))), int(np.floor(scale_factor_inv*(x-border_w)))]
        centre = small_img[int(y-border_w), int(x-border_w)]

        return tuple(centre),debug

    def getObjectMask(self, img):
        # Separate buoy cluster and make white
        thres = 10

        centre_colour_low  = np.subtract(self.centre_colour, (thres, thres, thres))
        centre_colour_high = np.add(self.centre_colour,      (thres, thres, thres))
        object_mask = cv2.inRange(img, centre_colour_low, centre_colour_high)

        return object_mask

    def raw_moment(self, data, i_order, j_order):
        nrows, ncols = data.shape
        y_indices, x_indicies = np.mgrid[:nrows, :ncols]
        return (data * x_indicies**i_order * y_indices**j_order).sum()

    def moments_cov(self, data):
        data_sum = data.sum()
        m10 = self.raw_moment(data, 1, 0)
        m01 = self.raw_moment(data, 0, 1)
        x_centroid = m10 / data_sum
        y_centroid = m01 / data_sum
        u11 = (self.raw_moment(data, 1, 1) - x_centroid * m01) / data_sum
        u20 = (self.raw_moment(data, 2, 0) - x_centroid * m10) / data_sum
        u02 = (self.raw_moment(data, 0, 2) - y_centroid * m01) / data_sum
        cov = np.array([[u20, u11], [u11, u02]])
        return (int(x_centroid), int(y_centroid)), cov

    def rotateCropScale(self, img):
        # Crop image to buoy
        cont_return = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

        if len(contours) == 0:
            #print("Couldn't find contours for image mask. Mask might not include a buoy.")
            return None, None

        best_cnt = max(contours, key=cv2.contourArea) # Get largest contour
        x, y, w, h = cv2.boundingRect(best_cnt)
        cropped_img = img[y:y+h, x:x+w] # Crop rectangle around buoy

        # Get rotation angle to vertically align buoy
        center, cov_matrix = self.moments_cov(cropped_img)
        eig_vals, eig_vecs = np.linalg.eig(cov_matrix)
        sort_indices = np.argsort(eig_vals)[::-1] # Index of largest eigenvalue
        x_v1, y_v1 = eig_vecs[:, sort_indices[0]] # Eigenvector with largest eigenvalue
        theta = np.tanh((x_v1)/(y_v1))            # Angle of largest eigenvector

        # Rotate image to align with major axis of buoy
        rot_mat = cv2.getRotationMatrix2D(center, np.degrees(-theta), 1.0)
        largest_dim  = max(cropped_img.shape)
        rotated_img = cv2.warpAffine(cropped_img, rot_mat, (largest_dim,
                 largest_dim))

        # Crop image again, since rotation changes positioning
        cont_return = cv2.findContours(rotated_img, cv2.RETR_EXTERNAL,
                 cv2.CHAIN_APPROX_SIMPLE)
        contours = cont_return[0] if len(cont_return) is 2 else cont_return[1] # Version fix

        best_cnt = max(contours, key=cv2.contourArea) # Get largest contour
        x, y, w, h = cv2.boundingRect(best_cnt)
        cropped_img = rotated_img[y:y+h, x:x+w] # Crop rectangle around buoy

        # Scale image to constant size
        width = 100
        scaled_img = cv2.resize(cropped_img, (width, width))
        return scaled_img, w

    def mirrorCombine(self, img):
        flipped_img = cv2.flip(img, 1)
        mirrored_img = cv2.bitwise_or(img, flipped_img)
        return mirrored_img

    def normalise(self, colour):
        ''' Normalises RGB colour. '''
        colour_sum = sum(colour)
        normalised_colour = (int(255*colour[0]/colour_sum),
            int(255*colour[1]/colour_sum), int(255*colour[2]/colour_sum))

        gray_tolerance = 21 # Check if this is a grayscale colour
        colour_average = int(float(colour_sum)/3)
        sum_gray_error = 0
        for component in colour:
            sum_gray_error += np.absolute(component - colour_average)

        if sum_gray_error < gray_tolerance: # We have a grayscale
            if colour_sum < 150: # < (50, 50, 50) black
                return (0, 0, 0)
            else:                # > (50, 50, 50) white
                return (255, 255, 255)
        else:
            return normalised_colour # Forget grayscale, just return normalised

    def getPolyformType(self, obj_width, distance):

        suffixes   = ["_a3", "_a5", "_a7"]
        #radii     = [0.238, 0.370, 0.550] # Polyform radii in metres
        thresholds = [0.280, 0.440]        # Original [0.304, 0.460]

        theta = np.arctan((float(obj_width)/2)/self.focal_length)
        polyform_radius = float(distance)*np.sin(theta)

        return_label = "polyform"

        if polyform_radius < thresholds[0]:   # a3
            return_label += suffixes[0]
        elif polyform_radius < thresholds[1]: # a5
            return_label += suffixes[1]
        else:                                 # a7
            return_label += suffixes[2]

        return return_label

    def classify(self, img, distance):

        clustered_img = self.kMeans(img)
        self.centre_colour, debug = self.centreColour(clustered_img)

        if self.centre_colour is not None:
            object_mask            = self.getObjectMask(clustered_img)
            cropped_img, obj_width = self.rotateCropScale(object_mask)

            if cropped_img is not None:
                mirrored_img = self.mirrorCombine(cropped_img)

                # Classify the buoy
                label, conf_shape, conf_colour = self.classifyImage(mirrored_img, self.bgr2rgb(self.centre_colour))

                # If polyform, narrow down size
                if label == "polyform":
                    label = self.getPolyformType(obj_width, distance)

                if distance < 15:
                    dist_scale= 1
                else:
                    dist_scale = 1 - ((distance-15)/60)

                rospy.logdebug("Label: %s\nShape Confidence: %s\nColour Confidence: %s\n" , label, conf_shape, conf_colour)
                return label, conf_shape*conf_colour*dist_scale, clustered_img
            else:
                rospy.logdebug("No cropped image returned")
                return "", 0.0, clustered_img
        else:
            rospy.logdebug("No centre colour return:")
            return "", 0.0, clustered_img
