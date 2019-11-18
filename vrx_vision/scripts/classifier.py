import numpy as np
import cv2
import rospkg

class Classifier(object):

    def __init__(self):
        self.template_filename_list = []
        self.template_labels = []
        self.template_colours = []

    def bgr2rgb(self, colour):
        ''' Converts colour from BGR to RGB. '''
        rgb_colour = (colour[2], colour[1], colour[0])
        return rgb_colour

    def normalise(self, colour):
        ''' Normalises RGB colour. '''
        colour_sum = sum(colour)
        normalised_colour = (int(255*colour[0]/colour_sum),
            int(255*colour[1]/colour_sum), int(255*colour[2]/colour_sum))
        return normalised_colour

    def colourConfidenceRGB(self, colour1, colour2):
        ''' Calculates Euclidean distance between two RGB colours. Returns distance as
            percentage of maximum possible distance.
        '''
        max_dist = 441.673 # Maximum possible RGB colour distance
        colour1 = self.normalise(colour1)
        colour2 = self.normalise(colour2)

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

    def classifyImage(self, img, colour):
        ''' Compares input img with template library and returns string name
            of highest confidence match type, along with confidence. Colour is used to
            determine exact model.

            return (string, float, float): (label, conf_label, conf_colour), where
                confidences are from 0 to 1.
        '''
        label_confidences  = []
        colour_confidences = []


        for template_filename in self.template_filename_list: # Compare img with templates
            template_img = cv2.imread(template_filename, cv2.IMREAD_GRAYSCALE)
            label_confidences.append(self.percentSimilar(template_img, img))

        print(label_confidences)        
        """conf_shape       = max(label_confidences) # Get highest confidence label and index
        best_shape_index = np.argmax(label_confidences)

        for template_colour in self.template_colours[best_shape_index]: # Compare colour with templates
            colour_confidences.append(self.colourConfidenceRGB(template_colour, colour))

        conf_colour       = max(colour_confidences) # Get highest confidence colour and index
        best_colour_index = np.argmax(colour_confidences)

        label = self.template_labels[best_shape_index][best_colour_index] # Get best label
        return label, conf_shape, conf_colour"""

        max_conf = 0
        max_colour_conf = 0
        max_label_conf = 0
        label = ""
        for conf_shape in label_confidences:
            index = label_confidences.index(conf_shape)
            for template_colour in self.template_colours[index]: # Compare colour with templates
                total_conf = self.colourConfidenceRGB(template_colour, colour)*conf_shape
                index2 = self.template_colours[index].index(template_colour)
                if total_conf > max_conf:
                    max_conf = total_conf
                    max_colour_conf = self.colourConfidenceRGB(template_colour, colour)
                    max_label_conf = conf_shape
                    label = self.template_labels[index][index2]

        return label,max_label_conf,max_colour_conf