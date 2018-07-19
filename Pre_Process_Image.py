from __future__ import division

import argparse as ap
import numpy as np
import interceptHelper as iH
import cv2
from skimage import io

# Get the path of the training set
parser = ap.ArgumentParser()
parser.add_argument("-i", "--image", help="Path to Image", required="True")
args = vars(parser.parse_args())

# Load the image
img = cv2.imread(args["image"])
img = img.copy()

mask = io.imread("Background.jpg")

# img_filtered = iH.rm_background(img, mask)

img_filtered = cv2.subtract(mask, img)

img_filtered = iH.rm_shadow(img_filtered)

# cv2.imshow("Removed Background", img_filtered)
# cv2.imshow("Mask Image", mask)
# cv2.waitKey()
kernel = np.ones((1, 1), np.uint8)
img_erosion = cv2.erode(img_filtered, kernel, iterations=2)
# img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)
# 20 50 50
# img_blurred_bilateral = cv2.bilateralFilter(img_erosion, 9, 75, 75)

# img_blurred_bilateral = img_filtered'

#################################################
# img_blurred = cv2.blur(img_filtered, (5,5))
# cv2.imshow("Image Denoised", img_blurred)
#################################################
# kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])/2
# im = cv2.filter2D(img_blurred_bilateral, -1, kernel)
# cv2.imshow("sharpened Image", im)
# img_blurred_bilateral = im
#################################################
# blur = cv2.GaussianBlur(img_filtered,(3,3),0)
# smooth = cv2.addWeighted(blur,1.5,img_filtered,-0.5,0)
# cv2.imshow("Refine Edges", smooth)
################################################
kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)
cv2.imshow("Closing", closing)
cv2.waitKey()

# cv2.imshow("Preprocessed Image", img_blurred_bilateral)
# cv2.waitKey()
edges = cv2.Canny(closing, 200, 200)
# edges = cv2.Canny(img_filtered, 150, 200)

cv2.imshow("Canny Edges", edges)
# lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=20, maxLineGap=60)
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=10, maxLineGap=40)

unext_img = img.copy()

for new_line in lines:
    # Draw Lines after extension
    cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

cv2.imshow("Displaying Result", unext_img)
cv2.waitKey()