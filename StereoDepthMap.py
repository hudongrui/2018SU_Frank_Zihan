from __future__ import division

import argparse as ap
import numpy as np
import cv2

# # Get the path of the training set
# parser = ap.ArgumentParser()
# parser.add_argument('-i', "--image_left", help="Path to Image", required="True")
# parser.add_argument('-i', "--image_right", help="Path to Image", required="True")
# args = vars(parser.parse_args())

# # Load the image
# imgL = cv2.imread(args["image_left"])
# imgR = cv2.imread(args["image_right"])

# Load the image
imgL = cv2.imread('ambush_5_left.jpg', 0)
imgR = cv2.imread('ambush_5_right.jpg', 0)


stereo = cv2.StereoBM_create(numDisparities=512, blockSize=11)
disparity = stereo.compute(imgL, imgR)
cv2.imshow("Left Image", imgL)
cv2.imshow("Right Image", imgR)
cv2.imshow("Disparity Map", disparity)
cv2.waitKey()