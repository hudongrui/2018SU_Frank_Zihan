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


img_filtered = cv2.subtract(mask, img)

img_filtered = iH.rm_shadow(img_filtered)

# cv2.imshow("Removed Background", img_filtered)
# cv2.imshow("Mask Image", mask)
# cv2.waitKey()
# kernel = np.ones((1, 1), np.uint8)
# img_erosion = cv2.erode(img_filtered, kernel, iterations=2)

kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)
cv2.imshow("Closing", closing)
# cv2.waitKey()

contrast = iH.increase_contrast(closing)
cv2.imshow("Increast Contrast", contrast)
# cv2.waitKey()

edges = cv2.Canny(contrast, 200, 200)
# edges = cv2.Canny(img_filtered, 150, 200)

cv2.imshow("Canny Edges", edges)
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=12, maxLineGap=25)

unext_img = img.copy()

for new_line in lines:
    # Draw Lines after extension
    cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)
    # cv2.line(unext_img, new_line[0],new_line[1], (0, 0, 255), 1)

cv2.imshow("Displaying Result", unext_img)
cv2.waitKey()