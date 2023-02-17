import cv2
import os
import numpy as np
import time
import matplotlib.pyplot as plt

#open background image
file_path = os.path.abspath(os.path.dirname(__file__))
background = cv2.imread(os.path.realpath(file_path + "/Assets/black.png"))

print(np.shape(background))

cv2.rectangle(background, pt1=(10,10), pt2=(200, 200), color=(255,0,0), thickness=10)





#show and wait
cv2.imshow("",background)
cv2.waitKey()