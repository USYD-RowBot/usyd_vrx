import cv2
import time
from objhelper.buoy_classifier import BuoyClassifier

crop_img = cv2.imread('red_buoy.png')
classifier = BuoyClassifier()
start = time.time()
print(start)
type, confidence = classifier.classify(crop_img, 100)
# run your code
end = time.time()

elapsed = end - start

#0.64sec for original code
#
