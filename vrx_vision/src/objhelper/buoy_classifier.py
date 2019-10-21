import numpy as np
import cv2
import rospkg

class BuoyClassifier():

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
        rows, columns, _ = np.shape(img)
        centre = img[int(rows/2), int(columns/2)]
        return tuple(centre)

    def getObjectMask(self, img):
        # Separate buoy cluster and make white
        centre_colour      = self.centreColour(img)
        centre_colour_low  = np.subtract(centre_colour, (1, 1, 1))
        centre_colour_high = np.add(centre_colour, (1, 1, 1))
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
        _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
        rotated_img = cv2.warpAffine(cropped_img, rot_mat, (largest_dim, largest_dim))

        # Crop image again, since rotation changes positioning
        _, contours, _ = cv2.findContours(rotated_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt = max(contours, key=cv2.contourArea) # Get largest contour
        x, y, w, h = cv2.boundingRect(best_cnt)
        cropped_img = rotated_img[y:y+h, x:x+w] # Crop rectangle around buoy

        # Scale image to constant size
        width = 100
        scaled_img = cv2.resize(cropped_img, (width, width))
        return scaled_img

    def mirrorCombine(self, img):
        flipped_img = cv2.flip(img, 1)
        mirrored_img = cv2.bitwise_or(img, flipped_img)
        return mirrored_img

    def bgr2rgb(self, colour):
        ''' Converts colour from BGR to RGB. '''
        rgb_colour = (colour[2], colour[1], colour[0])
        return rgb_colour

    def normalise(self, colour):
        ''' Normalises RGB colour. '''

        colour_sum = sum(colour)
        if colour_sum > 0:
            normalised_colour = (int(255*colour[0]/colour_sum),
                int(255*colour[1]/colour_sum), int(255*colour[2]/colour_sum))
        else:
            normalised_colour = colour

        return normalised_colour

    def colourConfidenceRGB(self, colour1, colour2):
        ''' Calculates Euclidean distance between two RGB colours. Returns distance as
            percentage of maximum possible distance.
        '''
        max_dist = 441.673 # Maximum possible RGB colour distance
        colour1 = self.normalise(colour1)
        colour2 = self.normalise(colour2)
        #print("c1: %s, c2: %s" % (colour1, colour2))

        R_dist = colour1[0] - colour2[0]
        G_dist = colour1[1] - colour2[1]
        B_dist = colour1[2] - colour2[2]

        colour_dist = np.sqrt(R_dist**2 + G_dist**2 + B_dist**2)/max_dist
        colour_conf = 1 - colour_dist
        return colour_conf

    def percentSimilar(self, img1, img2):
        ''' Calculates percentage similarity between two identically shaped binary images.
        '''
        if (img1.shape[0] is not img2.shape[0]) and (img1.shape[1] is not img2.shape[1]):
            print("Images are different sizes! Cannot compute similarity. img1: %s, img2: %s."
                % (img1.shape, img2.shape))
            return

        n_pixels = img1.shape[0]*img1.shape[1]
        n_equal_pixels = 0

        for i in range (img1.shape[0]):
            for j in range(img1.shape[1]):
                if img1[i][j] == img2[i][j]:
                    n_equal_pixels += 1

        return float(n_equal_pixels)/float(n_pixels)

    def classifyBuoy(self, img, colour):
        ''' Compares input img with template library and returns string name
            of highest confidence match type, along with confidence. Colour is used to
            determine exact buoy model.

            return (string, float, float): (label, conf_label, conf_colour), where
                confidences are from 0 to 1.
        '''

        rospack = rospkg.RosPack()
        pre = rospack.get_path('vrx_vision')+"/template_images/"

        template_filename_list = [
            pre+"template_conical.png", pre+"template_tophat.png", pre+"template_totem.png", pre+"template_sphere.png"]

        # (106, 183, 150)
        template_colours = [
            [(169, 71, 65)],                                                   # Conical
            [(222, 222, 222), (106, 183, 150)],                                # Tophat: (white, green)
            [(255, 255, 0), (1, 1, 1), (4, 4, 255), (4, 255, 4), (255, 4, 4)], # Totem:  (yellow, black, blue, green, red)
            [(0, 0, 0)]                                                        # Sphere
        ]

        template_labels = [
            ["surmark950410"],                                                        # Conical
            ["surmark46104", "surmark950400"],                                       # Tophat: (white, green)
            ["yellow_totem", "black_totem", "blue_totem", "green_totem", "red_totem"], # Totems
            ["polyform_a5"]                                                             # Sphere TODO estimate size
        ]

        label_confidences  = []
        colour_confidences = []

        for template_filename in template_filename_list: # Compare img with templates
            template_img = cv2.imread(template_filename, cv2.IMREAD_GRAYSCALE)
            label_confidences.append(self.percentSimilar(template_img, img))

            #cv2.imshow('K-Means Clustering', template_img)
            #cv2.waitKey(0)
        #print(label_confidences)
        conf_shape       = max(label_confidences) # Get highest confidence label and index
        best_shape_index = np.argmax(label_confidences)

        for template_colour in template_colours[best_shape_index]: # Compare colour with templates
            colour_confidences.append(self.colourConfidenceRGB(template_colour, colour))

        conf_colour       = max(colour_confidences) # Get highest confidence colour and index
        best_colour_index = np.argmax(colour_confidences)

        label = template_labels[best_shape_index][best_colour_index] # Get best label
        return label, conf_shape, conf_colour

    def classify(self, img, distance):

        clustered_img = self.kMeans(img)
        object_mask   = self.getObjectMask(clustered_img)
        cropped_img   = self.rotateCropScale(object_mask)
        mirrored_img  = self.mirrorCombine(cropped_img)

        # Classify the buoy
        label, conf_shape, conf_colour = self.classifyBuoy(mirrored_img, self.bgr2rgb(self.centreColour(img)))
        #print("Label: %s\nShape Confidence: %s\nColour Confidence: %s" % (label, conf_shape, conf_colour))

        #cv2.imshow('Shape',mirrored_img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        return label, conf_shape*conf_colour
