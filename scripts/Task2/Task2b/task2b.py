#! ~/cv_venv/bin/python3

######## importing libraries ###########
import cv2
import numpy as np


IMAGE_FILE = "yellow_detect.jpeg" # path of image file

img = cv2.imread(IMAGE_FILE) # importing image

img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) # converting to HSV colour space for colour detection

lower = np.array([0,140,87]) # getting array of lower bounds values
upper = np.array([57,255,255]) # getting array of upper bounds values

mask = cv2.inRange(img_hsv,lower,upper) # masking the yellow box region

cont, hier = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) # finding the contours of the yellow block

yellow_block_contour = max(cont, key=cv2.contourArea) # just a safe check

# calculating center of area using area moments
M = cv2.moments(yellow_block_contour)
cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

print(cX,cY) # printing the coordinates



# cv2.drawContours(img,[yellow_block_contour],-1,(0,0,255),2) 
# cv2.circle(img, (cX,cY), 5, (0,0,255), -1)
# cv2.imshow("output", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()