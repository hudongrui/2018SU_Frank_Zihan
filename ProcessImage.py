from __future__ import division

import argparse as ap
import numpy as np
import interceptHelper as iH
import cv2
from skimage import io


##################################################################
# TODO: Remove Shadow of the blocks
# TODO: Improve Extend Lines Method
# TODO: Improve Accuracy for finding edges shared by two blocks
# TODO: Allow Debug Mode to display picture
##################################################################

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

kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)
# cv2.imshow("Closing", closing)
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

cv2.imshow("Originally detected lines", unext_img)

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

    # cv2.putText(ext_img, str(line_cnt), (c_x,c_y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,0), 2)
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

# x, y, theta, bol = iH.check_intersect(ext_lines[3][0], ext_lines[6][0])
#
# if bol:
#     print("Found intersection at (" + str(x) + ", " + str(y) + ")")

intersections = iH.rm_nearby_intersect(intersections)
found_rect = iH.categorize_rect(intersections)
found_rect_centers = iH.rm_duplicates(found_rect, intersections)

# Remove intersections that are formed by two adjacent blocks located roughly one block away
found_rect_centers = iH.rm_false_positive(found_rect_centers, contrast)

# Display Results
number_of_center = 0
height, width, _ = img.shape
blank_image = np.zeros((height, width, 3), np.uint8)

for point in intersections:
    cv2.circle(blank_image, (point.x, point.y), 5, (255, 255, 255), -1)
for center in found_rect_centers:
    number_of_center += 1
    cv2.circle(blank_image, (int(center.x), int(center.y)), 7, (0, 255, 255), -1)
print("Found " + str(len(found_rect_centers)) + " blocks in the frame")
if number_of_center == 0:
    print("Could not find any blocks.")

cv2.imshow("Only the dots", blank_image)
cv2.waitKey()