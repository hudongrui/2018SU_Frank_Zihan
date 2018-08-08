from __future__ import division

import argparse as ap
import numpy as np
import interceptHelper as iH
import cv2
from skimage import io
import time


##################################################################
# TODO: Remove Shadow of the blocks
# TODO: Improve Accuracy for finding edges shared by two blocks
# TODO: Check Intersections Run-Time
# TODO Henry:
#   decrease dependency && improve the categorize rect method
#       using the diagonal output to debug
#
##################################################################

debug_mode = 1

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

kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)

contrast = iH.increase_contrast(closing)

edges = cv2.Canny(contrast, 200, 200)

lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=12, maxLineGap=25)

unext_img = img.copy()

for new_line in lines:
    # Draw Lines after extension
    cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

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

    line_cnt = line_cnt + 1

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

found_rect = iH.rm_duplicates(found_rect)
found_rect = iH.rm_close_to_intersects(found_rect, intersections)

# Remove intersections that are formed by two adjacent blocks located roughly one block away
found_rect = iH.rm_false_positive(found_rect, contrast)

# Display Results
number_of_center = 0
height, width, _ = img.shape
unproc_image = img.copy()


rect_cnt = 0
for index in found_rect:
    number_of_center += 1
    cv2.circle(unproc_image, (int(index.center.x), int(index.center.y)), 4, (0, 255, 255), -1)
    cv2.putText(unproc_image, str(rect_cnt + 1), (int(index.center.x + 5), int(index.center.y - 20)), cv2.FONT_HERSHEY_COMPLEX, 0.5, (50, 200, 200), 1)
    rect_cnt = rect_cnt + 1


height, width, _ = img.shape
blank_image = np.zeros((height,width,3), np.uint8)
for point in intersections:
    cv2.circle(blank_image, (point.x, point.y), 5, (255, 255, 255), -1)


# Draw the center of found rectangles
for index in found_rect:
    cv2.circle(blank_image, (int(index.center.x), int(index.center.y)), 7, (0, 255, 255), -1)


print("Found " + str(len(found_rect)) + " blocks in the frame")
if number_of_center == 0:
    print("Could not find any blocks.")

# In Debug Mode, display all intermediate processes
if debug_mode == 1:

    cv2.imshow("Removed Background", img_filtered)
    cv2.imshow("Closing", closing)
    cv2.imshow("Increast Contrast", contrast)
    cv2.imshow("Canny Edges", edges)
    cv2.imshow("Originally detected lines", unext_img)
    cv2.imshow("Extend the lines", ext_img)
    cv2.imshow("Only the dots", blank_image)
    cv2.imshow("Detection Result", unproc_image)

    # for rect in found_rect:
    #     rect.drawDiagonal1(unproc_image)
    #     cv2.imshow("1", unproc_image)
    #     # cv2.waitKey()
    #     # cv2.destroyAllWindows()
    #
    #     rect.drawDiagonal2(unproc_image)
    #     cv2.imshow("2", unproc_image)
    #     # cv2.waitKey()
    #     # cv2.destroyAllWindows()


cv2.waitKey()
