from __future__ import division

import argparse as ap
import numpy as np
import interceptHelper as iH
import cv2
from skimage import io


##################################################################
# TODO: Remove Shadow of the blocks
# TODO: Improve Extend Lines Method
# TODO: Remove Duplicates
# TODO: Improve Accuracy for finding edges shared by two blocks
##################################################################

# Get the path of the training set
parser = ap.ArgumentParser()
parser.add_argument("-i", "--image", help="Path to Image", required="True")
args = vars(parser.parse_args())

# Load the image
img = cv2.imread(args["image"])
img = img.copy()

mask = io.imread("Background.jpg")

cv2.imshow("Original", img)

img_filtered = cv2.subtract(mask, img)

img_filtered = iH.rm_shadow(img_filtered)

# cv2.imshow("Removed Background", img_filtered)

img_shadowless = iH.rm_shadow(img)
kernel = np.ones((5, 5), np.uint8)
img_erosion = cv2.erode(img_filtered, kernel, iterations=1)
img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)
# 20 50 50
img_blurred_bilateral = cv2.bilateralFilter(img_dilation, 9, 75, 75)

# img_blurred_bilateral = img_filtered

# cv2.imshow("Preprocessed Image", img_blurred_bilateral)
# cv2.waitKey()
edges = cv2.Canny(img_blurred_bilateral, 50, 200)

cv2.imshow("Canny Edges", edges)
# lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=20, maxLineGap=60)
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=30, maxLineGap=40)

unext_img = img.copy()

for new_line in lines:
    # Draw Lines after extension
    cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

    # cv2.imshow("Originally detected lines", unext_img)

ext_lines = []
ext_img = img.copy()

line_cnt = 0

for line in lines.copy():
    new_line = iH.extend_line(line)
    ext_lines.append(new_line)

    # Draw Lines after extension
    cv2.line(ext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

    c_x = int((new_line[0][0] + new_line[0][2])/2)
    c_y = int((new_line[0][1] + new_line[0][3])/2)

    cv2.putText(ext_img, str(line_cnt), (c_x,c_y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,0), 2)
    line_cnt = line_cnt + 1
    cv2.imshow("Extend the lines", ext_img)

intersections = []
i = 0
for line_1 in ext_lines:
    j = 0
    for line_2 in ext_lines:
        if i < j:
            x_center, y_center, theta, found = iH.check_intersect(line_1[0], line_2[0])
            if found:
                new_point = iH.Intersect(x_center, y_center, theta=theta)
                intersections.append(new_point)
        j += 1
    i += 1

intersections = iH.rm_nearby_intersect(intersections)
found_rect = iH.categorize_rect(intersections)
found_rect_centers = iH.rm_duplicates(found_rect, intersections)

number_of_center = 0
height, width, _ = img.shape
blank_image = np.zeros((height, width, 3), np.uint8)
for point in intersections:
    cv2.circle(blank_image, (point.x, point.y), 5, (255, 255, 255), -1)
for center in found_rect_centers:
    number_of_center += 1
    cv2.circle(blank_image, (int(center[0]), int(center[1])), 7, (0, 255, 255), -1)
print(number_of_center)
if number_of_center == 0:
    print("Could not find any blocks.")

# cv2.imshow("Only the dots", blank_image)
cv2.imshow("Preprocessed Image", img_blurred_bilateral)
cv2.waitKey()