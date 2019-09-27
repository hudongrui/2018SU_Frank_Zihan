from __future__ import division
import math
import warnings
import cv2
import numpy as np
import time
import copy
from skimage import io
import sys
import random
import rospy
import GraspingHelperClass as Gp

##################################################################################
# debugMode helper
# -1    -- enable all debugging feature
# 0     -- disable debug
# 1     -- matrix debugging
# 2     -- edge detection debug
# 3     -- grasping angle debug

debugMode = 3


##################################################################################

#########################################################
# This file contains all methods related to block recognition
#########################################################


# Check intersection of two points, if there is return the
# point, angle, and True; if not, return none and False
def check_intersect(line_1, line_2):
    # Endpoints of the first line
    pt1 = (line_1[0], line_1[1])
    pt2 = (line_1[2], line_1[3])
    # Endpoints of the second line
    pt3 = (line_2[0], line_2[1])
    pt4 = (line_2[2], line_2[3])

    # Calculate slope and y-intersect of each line
    m1 = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    b1 = round(pt1[1] - pt1[0] * m1, 8)

    m2 = (pt4[1] - pt3[1]) / (pt4[0] - pt3[0])
    b2 = round(pt3[1] - pt3[0] * m2, 8)

    # Ignore warning when getting a infinity slope
    warnings.filterwarnings("ignore")

    # Consider if the lines are horizontal or vertical to cause a non-resolvable slope for intersection
    if abs(m1) == float('Inf') and abs(m2) <= 0.1:
        if pt3[0] <= pt1[0] <= pt4[0] and min(pt1[1], pt2[1]) <= pt3[1] <= max(pt1[1], pt2[1]):
            x_intersect = pt1[0]
            y_intersect = pt3[1]
            theta = 90
            return x_intersect, y_intersect, theta, True
    elif abs(m1) <= 0.1 and abs(m2) == float('Inf'):
        if pt1[0] <= pt3[0] <= pt2[0] and min(pt3[1], pt4[1]) <= pt1[1] <= max(pt3[1], pt4[1]):
            x_intersect = pt3[0]
            y_intersect = pt1[1]
            theta = 90
            return x_intersect, y_intersect, theta, True
    elif abs(m1 - m2) < 0.2:
        # print("Same Slope")
        return None, None, None, False

    solution = (b2 - b1) / (m1 - m2)

    # Check if intersects fall in the range of two lines
    if pt1[0] <= solution <= pt2[0] and pt3[0] <= solution <= pt4[0]:
        # print("Solution is " + str(float(solution[0])))

        x_intersect = int(solution)
        y_intersect = int(m2 * solution + b2)

        theta1 = math.atan(m1)
        theta2 = math.atan(m2)
        theta = int(math.degrees(theta2 - theta1))

        # Adjust the threshold angle below to check for perpendicular lines
        if (100 > theta > 80) or (-100 < theta < -80):
            return x_intersect, y_intersect, theta, True
        else:
            # print("Lines are not nearly perpendicular")
            return None, None, theta, False
    else:
        # print("Intersection is not within the lines")
        return None, None, None, False


# Remove similar lines in vacinity to each other that are being detected by Hough Transform
def rm_line_duplicates(lines):
    for line_1 in lines:
        pt1 = (line_1[0], line_1[1])
        pt2 = (line_1[2], line_1[3])
        # Calculate slope and y-intersect of each line
        m1 = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
        b1 = round(pt1[1] - pt1[0] * m1, 10)

        for line_2 in lines:
            # Endpoints of the second line
            pt3 = (line_2[0], line_2[1])
            pt4 = (line_2[2], line_2[3])
            m2 = (pt4[1] - pt3[1]) / (pt4[0] - pt3[0])
            b2 = round(pt3[1] - pt3[0] * m2, 10)

            if abs(m1 - m2) < 0.2 and abs(b1 - b2) < 50:
                lines.remove(line_2)

    return lines


# Extends the edges to construct intersections, which might be a block's corners
def extend_line(line):
    x1, y1, x2, y2 = line[0]
    length = int(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    # TODO: Adjust the following threshold to pass the lines
    one_block_len = 55
    delta_x = 0
    delta_y = 0
    ratio = float(abs(1.5 * (one_block_len - length) / length))
    # TODO: The non-zero constant appeared below are arbitrary defined ratio
    if 2 * one_block_len <= length <= 2.5 * one_block_len:
        # print("Two Blocks")
        ratio = float(abs(0.8 * (one_block_len - length) / length))
    elif 0.8 * one_block_len < length < 1.2 * one_block_len:
        delta_x = int(abs(x2 - x1) * 0.3)
        delta_y = int(abs(y2 - y1) * 0.3)
    elif length < 0.8 * one_block_len:
        ratio = float(abs((1.3 * one_block_len - length) / length))

    if delta_x == 0 and delta_y == 0:
        delta_x = int(abs(x2 - x1) * ratio)
        delta_y = int(abs(y2 - y1) * ratio)
    x1_p = x1 - delta_x
    x2_p = x2 + delta_x
    if y1 > y2:
        y1_p = y1 + delta_y
        y2_p = y2 - delta_y
    else:
        y1_p = y1 - delta_y
        y2_p = y2 + delta_y
    extended = [x1_p, y1_p, x2_p, y2_p]
    # print("Original Length is " + str(length))
    # Extended_Length = int(math.sqrt((x2_p - x1_p) ** 2 + (y2_p - y1_p) ** 2))
    # print("Extended Length is: " + str(Extended_Length))
    return [extended]


# It's all in the name
def increase_contrast(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    # cv2.imshow("lab", lab)

    # -----Splitting the LAB image to different channels-------------------------
    l, a, b = cv2.split(lab)
    # cv2.imshow('l_channel', l)
    # cv2.imshow('a_channel', a)
    # cv2.imshow('b_channel', b)

    # -----Applying CLAHE to L-channel-------------------------------------------
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    # cv2.imshow('CLAHE output', cl)

    # -----Merge the CLAHE enhanced L-channel with the a and b channel-----------
    limg = cv2.merge((cl, a, b))
    # cv2.imshow('limg', limg)

    # -----Converting image from LAB Color model to RGB model--------------------
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    return final

# Remove intersections that are too close to each other
def rm_nearby_intersect(intersections):
    if len(intersections) != 0:
        i = 0
        for point_1 in intersections:
            j = 0
            for point_2 in intersections:
                if i < j:
                    if is_in_range_of_a_circle(point_1, point_2, radius_threshold=30):
                        intersections.remove(point_2)
                j += 1
            i += 1
    return intersections


# This method is replaced by is_in_range_of_a_circle() method instead
# def is_near(ctr, intersects):
#     bol = False
#     for index in intersects:
#         # print(str(abs(ctr.x - index.y)) + " and " + str)
#         if is_in_range_of_a_circle(ctr, index.center, radius_threshold=15):
#             bol = True
#             break
#     return bol


# Input List of rectangle, Output is list of rectangle with duplicate removed
def rm_duplicates(rects):
    boo = False
    copy_of_list = copy.deepcopy(rects)
    try:
        output = [copy_of_list[0]]
        for rect in copy_of_list:
            for compare_item in output:
                if is_in_range_of_a_circle(rect.center, compare_item.center, radius_threshold=25):
                    boo = True
            if boo is False:
                output.append(rect)
            boo = False
        return output
    except (IndexError, TypeError):
        return None


def rm_close_to_intersects(rects, intersections):
    list = []
    boo = False
    try:
        for rect in rects:
            point_1 = rect.center
            for intersect in intersections:
                point_2 = intersect
                if is_in_range_of_a_circle(point_1, point_2, radius_threshold=27):
                    boo = True
            if boo is False:
                list.append(rect)
            boo = False
        return list
    except TypeError:
        return None


# Remove the shadow projected by the blocks by thresholding
def rm_shadow(image):
    image = image.copy()
    h, w = image.shape[0], image.shape[1]

    for y in range(h):
        for x in range(w):
            pixel = image[y, x]
            r, g, b = pixel[0], pixel[1], pixel[2]
            # This is the limiting threshold that differentiate black and noise
            lim = 105
            max_value = max([r, g, b])
            min_value = min([r, g, b])

            if r < lim and g < lim and b < lim:
                # Base on the fact that noise's rgb values stays around
                if max_value - min_value < 20:
                    image[y, x] = [0, 0, 0]

    return image


# This method is currently unused, substituted by cv2.subtract
# def rm_background(img, mask):
#     h, w, _ = img.shape
#     # print(str(img[h//2, w//2]))
#     for y in range(h):
#         for x in range(w):
#             p_img = img[y, x]
#             p_mask = mask[y, x]
#             r, g, b = [abs(i - j) for i, j in zip(p_img, p_mask)]
#             t = 40
#             if r < t and g < t and b < t:
#                 img[y, x] = [0, 0, 0]
#
#     return img

# Check the color of the predicted rectangle center, if it's too similar to background color, remove it
def rm_false_positive(rect_centers, img):
    img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_RGB2GRAY)
    centers = []
    try:
        for rect in rect_centers:
            # print(img_gray[int(rect.y), int(rect.x)])
            # print("at (" + str(int(rect.center.y)) + ", " + str(int(rect.center.x)) + ")")
            if np.any(img_gray[int(rect.center.y), int(rect.center.x)] > 10):
                centers.append(rect)
        return centers
    except TypeError:
        return None


# All the possible corners are constructed as an Intersect object
# theta is the angle between the two intersecting lines
class Intersect:
    def __init__(self, x_intersect, y_intersect, theta=None, category=None):
        self.x = x_intersect
        self.y = y_intersect
        if theta is not None:
            self.theta = theta
        if category is not None:
            self.category = category


# TODO: implement the intercept method into here to improve independence
class Line:
    def __init__(self, start_point, end_point):
        self.start = start_point
        self.end = end_point
        self.theta = math.atan2((start_point.y - end_point.y), (start_point.x - end_point.x))
        self.length = math.hypot((start_point.x - end_point.x), (start_point.y - end_point.y))

    def getThetaInRad(self):
        return self.theta

    def getThetaInDeg(self):
        return self.theta * 180 / math.pi


def is_in_range_of_a_circle(point1, point2, radius_threshold=None):
    if radius_threshold is None:
        radius_threshold = 20
    return distance_between_points(point1, point2) < radius_threshold


def distance_between_points(point1, point2):
    return math.hypot((point2.x - point1.x), (point2.y - point1.y))


def categorize_rect(intersections):
    # start_time = time.time()
    tmp_center_list = []
    list_of_squares = []
    tmp_intersection = intersections
    for starting_point in tmp_intersection:
        for next_point in tmp_intersection:
            if starting_point != next_point:
                base_line = Line(starting_point, next_point)
                standard_length = base_line.length
                if standard_length <= 30:
                    break
                possible_1 = Intersect(starting_point.x - math.sin(base_line.theta) * base_line.length, starting_point.y
                                       + math.cos(base_line.theta) * base_line.length)
                possible_1_c = Intersect(next_point.x - math.sin(base_line.theta) * base_line.length, next_point.y
                                         + math.cos(base_line.theta) * base_line.length)
                possible_2 = Intersect(starting_point.x + math.sin(base_line.theta) * base_line.length, starting_point.y
                                       - math.cos(base_line.theta) * base_line.length)
                possible_2_c = Intersect(next_point.x + math.sin(base_line.theta) * base_line.length, next_point.y
                                         - math.cos(base_line.theta) * base_line.length)
                midPoint = mid_point(starting_point, next_point)
                possible_3 = Intersect(midPoint.x - math.sin(base_line.theta) * base_line.length / 2, midPoint.y +
                                       math.cos(base_line.theta) * base_line.length / 2)
                possible_3_c = Intersect(midPoint.x + math.sin(base_line.theta) * base_line.length, midPoint.y
                                         - math.cos(base_line.theta) * base_line.length)
                for third_point in tmp_intersection:
                    if is_in_range_of_a_circle(possible_1, third_point, standard_length * 0.3):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_1_c, forth_point, standard_length * 0.3):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
                    if is_in_range_of_a_circle(possible_2, third_point, standard_length * 0.3):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_2_c, forth_point, standard_length * 0.3):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
                    if is_in_range_of_a_circle(possible_3, third_point, standard_length * 0.3):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_3_c, forth_point, standard_length * 0.3):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
    list_of_squares = sorted(list_of_squares, key=lambda rect: rect.side_length)
    temp_list = copy.deepcopy(list_of_squares)
    try:
        standard_length = temp_list[0].side_length
        for rect in temp_list:
            if rect.side_length > standard_length * 1.8:
                temp_list.remove(rect)
        temp_list = rm_duplicates(temp_list)
        index = 0
        for rect in temp_list:
            rect.setIndex(index)
            index += 1
        # elapsed_time = time.time() - start_time
        # print("the time elapsed for categorizing square is " + str(elapsed_time))
        return temp_list
    except IndexError:
        return None

    # list_of_squares = sorted(list_of_squares, key=lambda rect: rect.getDistance())
    # temp_list = copy.deepcopy(list_of_squares)
    temp_list = []
    # standard_length = temp_list[0].getDistance()
    # print("the length of the smalles square is: ", standard_length)
    # print("detect ", len(list_of_squares), " of squares")
    for rect in list_of_squares:
        # if 60 < rect.distance < 90:
        if rect.distance < 10:
            temp_list.append(rect)
    # print("there are ", len(temp_list), " left after removing 0 length")

    list_of_squares = rm_duplicates(list_of_squares)
    return list_of_squares


# TODO: fix this bug -- Zihan
def append_rec_list(output_list, rect, center_list):
    output_list.append(rect)
    center_list.append(rect.center)


def mid_point(point1, point2):
    return Intersect((point1.x + point2.x) / 2, (point1.y + point2.y) / 2)


def refine_range(angle, lowerLimit=None, upperLimit=None, modifier=None):
    # method used for changing angle into range, could be used in other situation
    if lowerLimit is None:
        lowerLimit = -90
    if upperLimit is None:
        upperLimit = 90
    if modifier is None:
        modifier = 180
    while angle > upperLimit or angle < lowerLimit:
        if angle < lowerLimit:
            angle += modifier
        elif angle > upperLimit:
            angle -= modifier
    return angle


# Based on the intersection found, a rectangle object is constructed recording its location and pivot points
class Rectangle:
    def __init__(self, point1, point2, point3, point4=None, index=None, location=None):
        # location should be a 3-dimensional matrix that includes x, y, z coordinates of the block
        self.center = Intersect(0, 0)
        if index is not None:
            self.index = index
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        if distance_between_points(point1, point2) >= distance_between_points(point1, point3):
            self.side_length = distance_between_points(point1, point3)
        else:
            self.side_length = distance_between_points(point1, point2)
        if point4 is None:
            self.center = self.find_its_center_3()
        else:
            self.point4 = point4
            self.p = [point1, point2, point3, point4]
            self.center = self.find_its_center_4()
        if location is not None:
            self.location = location

    def find_its_center_3(self):
        length1 = distance_between_points(self.point1, self.point2)
        length2 = distance_between_points(self.point2, self.point3)
        length3 = distance_between_points(self.point1, self.point3)
        if length1 >= length2 and length1 >= length3:
            center = mid_point(self.point1, self.point2)
        elif length2 >= length1 and length2 >= length3:
            center = mid_point(self.point2, self.point3)
        else:
            center = mid_point(self.point1, self.point3)
        return center

    def find_its_center_4(self):
        x = [p.x for p in self.p]
        y = [p.y for p in self.p]
        return Intersect(sum(x) / len(x), sum(y) / len(y))

    def getLocation(self):
        return self.location

    def getCenterX(self):
        return self.center.x

    def getCenterY(self):
        return self.center.y

    def getCenter(self):
        return self.center

    def setLocation(self, pCoordinate, qCoordinate):
        self.location = {"p_x": pCoordinate[0], "p_y": pCoordinate[1], "p_z": pCoordinate[2],
                         "q_x": qCoordinate[0], "q_y": qCoordinate[1], "q_z": qCoordinate[2], "q_w": qCoordinate[3]}

    def setIndex(self, index):
        self.index = index

    def getDistance(self):
        return self.distance

    def getAngle(self, list_of_square):
        min_distance = sys.maxsize
        if len(list_of_square) > 1:
            for rect in list_of_square:
                other_center = rect.center
                if self.center.x != other_center.x and self.center.y != other_center.y:
                    new_distance = distance_between_points(rect.center, other_center)
                    if new_distance < min_distance:
                        min_distance = new_distance
                        min_distance_center = other_center
            correction_line = Line(self.center, min_distance_center)
        else:
            correction_line = Line(self.center, Intersect(0, 0))

        line1 = Line(self.point1, self.point2)
        line2 = Line(self.point1, self.point3)
        theta1 = refine_range(line1.getThetaInRad(), -math.pi / 2, math.pi / 2, math.pi)
        theta2 = refine_range(line2.getThetaInRad(), -math.pi / 2, math.pi / 2, math.pi)
        if debugMode == 3:
            if abs(theta1) > abs(theta2):
                output = theta2
            else:
                output = theta1
        else:
            correction_theta = refine_range(correction_line.getThetaInRad(), -math.pi / 2, math.pi / 2, math.pi * 2)
            if abs(theta1 - correction_theta) < abs(theta2 - correction_theta):
                output = theta2
            else:
                output = theta1
        return output

    def drawDiagonal1(self, image):
        color = (255, 0, 255)
        cv2.line(image, (self.point1.x, self.point1.y), (self.point4.x, self.point4.y), color, 1)

    def setIndex(self, num):
        self.index = num

    def drawDiagonal2(self, image):
        color = (255, 0, 255)
        cv2.line(image, (self.point2.x, self.point2.y), (self.point3.x, self.point3.y), color, 1)


# All in the name. Find this block helps to avoid picking up blocks under the influence of camera angle distortion
def find_square_closest_to_center(img, square_list):
    h, w, _ = img.shape
    ctr_x, ctr_y = int(w / 2), int(h / 2)
    dists = []

    for sq in square_list:
        manhattan_dist = abs(int(sq.getCenterX() - ctr_x)) + abs(int(sq.getCenterY() - ctr_y))
        dists.append(manhattan_dist)

    toReturn = square_list[dists.index(min(dists))]

    return toReturn


#########################################################################################
# TODO: This is the main code to perform block recognition
#
# This algorithm uses Canny Edge Detection and Hough Line transform to identify possible sides
# of the blocks. Then, extend the sides to form intersections. We assume all intersections are
# possible corners of the block and reconstruct rectangle-shaped blocks based on them. Finally,
# Check and remove rectangles that are falsely identified as blocks.
#
# NOTE: All the commented code blocks appeared in the method below are for debugging purpose
# Uncomment them to display intermediate outputs
#########################################################################################
def square_img_to_centers_list(img, workspace, square=None):
	#If using the Kinect camera, add _Kinect to the end of the file name.
	#If using the webcam, make sure the name doesn't have _Kinect on the end
    if workspace:
        # Right
        mask = io.imread("Background_Right.jpg")
    else:
        mask = io.imread("Background_Left.jpg")

    if square is not None:
        # For Second Time to Perform Recognition, crop only the center of the image to avoid picking up blocks with angle distortion
        ctr_x = square.getCenterX()
        ctr_y = square.getCenterY()
        dim = int(2 * square.side_length)

        x = int(ctr_x - 0.5 * dim)
        y = int(ctr_y - 0.5 * dim)
        l = dim
        img = img[y: y + l, x:x + l]
        mask = mask[y: y + l, x:x + l]

    img_filtered = cv2.subtract(mask, img)

    img_filtered = rm_shadow(img_filtered)

    # cv2.imshow("Removed Background", img_filtered)
    # cv2.imshow("Mask Image", mask)
    # cv2.waitKey()

    kernel = np.ones((5, 5), np.uint8)
    closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("Closing", closing)
    # cv2.waitKey()

    contrast = increase_contrast(closing)
    # cv2.imshow("Increast Contrast", contrast)
    # cv2.waitKey()

    edges = cv2.Canny(contrast, 150, 200)
    # edges = cv2.Canny(img_filtered, 150, 200)

    # cv2.imshow("Canny Edges", edges)
    # cv2.waitKey()
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=12, maxLineGap=25)

    unext_img = img.copy()

    for new_line in lines:
        # Draw Lines after extension
        cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

    # cv2.imshow("Originally detected lines", unext_img)
    # cv2.waitKey()

    ext_lines = []
    ext_img = img.copy()

    line_cnt = 0

    for line in lines.copy():
        new_line = extend_line(line)
        ext_lines.append(new_line)

        # Draw Lines after extension
        cv2.line(ext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

        c_x = int((new_line[0][0] + new_line[0][2]) / 2)
        c_y = int((new_line[0][1] + new_line[0][3]) / 2)

        # cv2.putText(ext_img, str(line_cnt), (c_x,c_y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,0), 2)
        line_cnt += 1
    # Optional: Remove Duplicate Lines for Robustness
    # ext_lines = iH.rm_line_duplicates(ext_lines)
    # cv2.imshow("Extend the lines", ext_img)

    intersections = []
    i = 0
    for line_1 in ext_lines:
        j = 0
        for line_2 in ext_lines:
            if i < j:
                x_center, y_center, theta, found = check_intersect(line_1[0], line_2[0])
                if found:
                    new_point = Intersect(x_center, y_center, theta=theta)
                    intersections.append(new_point)
            j += 1
        i += 1

    # x, y, theta, bol = iH.check_intersect(ext_lines[3][0], ext_lines[6][0])
    #
    # if bol:
    #     print("Found intersection at (" + str(x) + ", " + str(y) + ")")

    intersections = rm_nearby_intersect(intersections)
    found_rect = categorize_rect(intersections)
    # print("Found " + str(len(found_rect)) + "Rectangle")

    found_rect = rm_duplicates(found_rect)
    found_rect = rm_close_to_intersects(found_rect, intersections)

    # Remove intersections that are formed by two adjacent blocks located roughly one block away
    found_rect = rm_false_positive(found_rect, contrast)

    # Display Results
    number_of_center = 0
    height, width, _ = img.shape
    blank_image = img.copy()
    if found_rect is not None:
    	print("Found " + str(len(found_rect)) + " blocks in the frame")


    # for point in intersections:
    #     cv2.circle(blank_image, (point.x, point.y), 5, (255, 255, 255), -1)

    # rect_cnt = 0
    # for index in found_rect:
    #     number_of_center += 1
    #     cv2.circle(blank_image, (int(index.center.x), int(index.center.y)), 7, (0, 255, 255), -1)
    #     cv2.putText(blank_image, str(rect_cnt + 1), (int(index.center.x + 5), int(index.center.y - 20)),
    #                 cv2.FONT_HERSHEY_COMPLEX, 0.5, (50, 200, 200), 1)
    #     rect_cnt = rect_cnt + 1

    # for rect in found_rect:
    #     rect.drawDiagonal1(blank_image)
    #     rect.drawDiagonal2(blank_image)
    #
    # cv2.imshow("Removed Background", img_filtered)
    # cv2.imshow("Closing", closing)
    # cv2.imshow("Increast Contrast", contrast)
    # cv2.imshow("Canny Edges", edges)
    # cv2.imshow("Originally detected lines", unext_img)
    # cv2.imshow("Displaying Result", blank_image)
    # cv2.imshow("Extend the lines", ext_img)
    # cv2.waitKey()
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()
    return found_rect


# def get_joint_angles(x, y, z, theta, dQ):
#     return None


# Calculate actual location in the base-centered coordinate
def get_location(list_of_coordinate, workspace):

    # Default value for workspace is false, indicating drop_off_location should be on the right
##################################################################################
#
# TODO: IMPORTANT!!!!! Below are the positions for the temporary setup to address robot unable to avoid collision
#
##################################################################################
    # 36 inch, 18 inch
    if workspace:
        # Workspace on human's left
        ctr_x, ctr_y, ctr_z = Gp.in_to_m(25 + 3.2), Gp.in_to_m(19), 0.1
    else:
        # Workspace on human's right
        ctr_x, ctr_y, ctr_z = Gp.in_to_m(25 + 3.2), Gp.in_to_m(-19), 0.1


    locations = []
    for loc in list_of_coordinate:
        px, py, pz = loc[0], loc[1], loc[2]
        theta = (loc[3]) / 180 * np.pi
        b_len = 0.053 * math.sqrt(2) + 0.01 # The additional factor compensates for extended length due to possible rotation
        b_height = 0.045
        x, y, z = ctr_x + px * b_len, ctr_y + py * b_len, ctr_z + pz * b_height
        locations.append([x, y, z, theta])

    return locations


# Program a series of location here for the robot to drop of the block
# We construct a new coordinate system where the center drop-off location is origin,
# with each unit length of one block length, which is 0.0045m
def drop_destinations(workspace):
    # Pyramid
    preset_pyramid = [[0, -1, 0, -90], [0, 0, 0, -90], [0, 1, 0, -90], [0, -0.5, 1, -90], [0, 0.5, 1, -90], [0, 0, 2, -90]]
    #########
    # Below are all the 2-D presets
    #########
    # Center of workspace
    preset_0 = [[0, 0, 0, 0]]
    # 45 degrees rotation clockwise
    preset_1 = [[0, 0, 0, 45]]
    # two_Blocks
    preset_3 = [[0, 0, 0, 30], [0, 2, 0, 0]]
    ######
    # four blocks
    ######
    four_block_presets = []
    p_1 = [[0, 0, 0, 0], [1, 1, 0, 10], [-1, -1, 0, 0], [0, 1, 0, 25]]
    p_2 = [[1, 0, 0, 30], [0, 1, 0, 20], [0, -1, 0, 45], [-1, 0, 0, -45]]
    p_3 = [[1, 1, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0], [-1, -1, 0, 0]]
    p_4 = [[0, 1, 0, 50], [0, 0, 0, 0], [1, 0, 0, 0], [-1, -1, 0, 0]]
    p_5 = [[-1, -1, 0, 0], [1, 1, 0, 0], [0, -1, 0, 60], [1, -1, 0, 80]]
    p_6 = [[0, 1, 0, 30], [1, -1, 0, 0], [1, 0, 0, -30], [0, 0, 0, 20]]
    four_block_presets.append(p_1)
    four_block_presets.append(p_2)
    four_block_presets.append(p_3)
    four_block_presets.append(p_4)
    four_block_presets.append(p_5)
    four_block_presets.append(p_6)

    if workspace:
        final_preset = p_2

    else:
        final_preset = p_3

    i = random.randint(0, len(four_block_presets) - 1)
    print "Assuming four blocks in work space, using preset " + str(i + 1)
    return get_location(preset_0, workspace)


def read_tasks_from_file():
	# Read all the image file
	task_img = []

	# TODO: For now there are 12 picture setup in the folder
	for i in range(1,13):
		file_name = "task_" + str(i) + ".jpg"
		task_img.append(io.imread(file_name))


	# Process each image to return a list of task
	task_coordinate = []
	for img in task_img:
		list_of_coordinates = []

		img_gray =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		_, binary_img = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

		# cv2.imshow("Gray_img", binary_img)
		# cv2.waitKey()

		height, width, _ = img.shape

		cnt_img, cnts, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)

		blank_img = np.zeros((height, width, 3), np.uint8)
		blank_img[:,:] = (255,255,255) 

		n = 0
		for i in range (len(cnts)):

			cnt = cnts[i]

			if cv2.contourArea(cnt) > 2000 and cv2.contourArea(cnt) < 5000 and hierarchy[0, i, 2] == -1:
				rect = cv2.minAreaRect(cnt)
				theta = refine_range(90 - rect[2], -90, 90, 180)

				# print str(n) + ' ' + str(rect)

				box = cv2.boxPoints(rect)
				box = np.int0(box)
				# cv2.drawContours(blank_img,[box],0,(255,0,0),2)

				# M = cv2.moments(cnt)
				# cx = int(M['m10']/M['m00'])
				# cy = int(M['m01']/M['m00'])
				cx = int(rect[0][0])
				cy = int(rect[0][1])

				x = cx - width/2
				y = cy - height/2
				z = 0
				list_of_coordinates.append([-y, -x, z, theta])
				n += 1
		
		# exit()		
		# cv2.imshow("Contour", blank_img)
		# cv2.waitKey()
		task_coordinate.append(list_of_coordinates)

	return task_coordinate

def get_loc_by_pixel(workspace, task):
	
	if workspace:
		# Workspace on human's left
		ctr_x, ctr_y, ctr_z = Gp.in_to_m(20 + 3.2), Gp.in_to_m(16), 0.1
	else:
	    # Workspace on human's right
	    ctr_x, ctr_y, ctr_z = Gp.in_to_m(20 + 3.2), Gp.in_to_m(-15.5), 0.1


	locations = []

	factor = Gp.in_to_m(0.035238) * 0.9

	for loc in task:	
	    px, py, z, theta = loc[0], loc[1], loc[2], loc[3]

	    theta = theta / 180 * np.pi

	    x, y, z = ctr_x + px * factor, ctr_y + py * factor, ctr_z + z
	    locations.append([x, y, z, theta])


	# locations = [[ctr_x,ctr_y,ctr_z,0]]

	return locations



# This the function for finding the marked area for drop off 
def get_marked_location(img, workspace, square = None):

    if workspace:
        # Right
        mask = io.imread("Background_Right.jpg")
    else:
        mask = io.imread("Background_Left.jpg")

    if square is not None:
        # For Second Time to Perform Recognition, crop only the center of the image to avoid picking up blocks with angle distortion
        ctr_x = square.getCenterX()
        ctr_y = square.getCenterY()
        dim = int(2 * square.side_length)

        x = int(ctr_x - 0.5 * dim)
        y = int(ctr_y - 0.5 * dim)
        l = dimread_task
        img = img[y: y + l, x:x + l]
        mask = mask[y: y + l, x:x + l]

    img_filtered = cv2.subtract(mask, img)

    img_filtered = rm_shadow(img_filtered)

    # cv2.imshow("Removed Background", img_filtered)
    # cv2.imshow("Mask Image", mask)
    # cv2.waitKey()

    kernel = np.ones((5, 5), np.uint8)
    closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("Closing", closing)
    # cv2.waitKey()

    contrast = increase_contrast(closing)
    # cv2.imshow("Increast Contrast", contrast)
    # cv2.waitKey()

    edges = cv2.Canny(contrast, 150, 200)
    # edges = cv2.Canny(img_filtered, 150, 200)

    # cv2.imshow("Canny Edges", edges)
    # cv2.waitKey()
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=12, maxLineGap=25)

    unext_img = img.copy()

    for new_line in lines:
        # Draw Lines after extension
        cv2.line(unext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

    # cv2.imshow("Originally detected lines", unext_img)
    # cv2.waitKey()

    ext_lines = []
    ext_img = img.copy()

    line_cnt = 0

    for line in lines.copy():
        new_line = extend_line(line)
        ext_lines.append(new_line)

        # Draw Lines after extension
        cv2.line(ext_img, (new_line[0][0], new_line[0][1]), (new_line[0][2], new_line[0][3]), (0, 0, 255), 1)

        c_x = int((new_line[0][0] + new_line[0][2]) / 2)
        c_y = int((new_line[0][1] + new_line[0][3]) / 2)

        # cv2.putText(ext_img, str(line_cnt), (c_x,c_y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,0), 2)
        line_cnt += 1
    # Optional: Remove Duplicate Lines for Robustness
    # ext_lines = iH.rm_line_duplicates(ext_lines)
    # cv2.imshow("Extend the lines", ext_img)
    # cv2.waitKey()

    intersections = []
    i = 0
    for line_1 in ext_lines:
        j = 0
        for line_2 in ext_lines:
            if i < j:
                x_center, y_center, theta, found = check_intersect(line_1[0], line_2[0])
                if found:
                    new_point = Intersect(x_center, y_center, theta=theta)
                    intersections.append(new_point)
            j += 1
        i += 1

    # x, y, theta, bol = iH.check_intersect(ext_lines[3][0], ext_lines[6][0])
    #
    # if bol:
    #     print("Found intersection at (" + str(x) + ", " + str(y) + ")")

    intersections = rm_nearby_intersect(intersections)
    found_rect = categorize_rect(intersections)
    # print("Found " + str(len(found_rect)) + "Rectangle")

    found_rect = rm_duplicates(found_rect)
    found_rect = rm_close_to_intersects(found_rect, intersections)

    # Remove intersections that are formed by two adjacent blocks located roughly one block away
    found_rect = rm_false_positive(found_rect, contrast)

    # Display Results
    number_of_center = 0
    height, width, _ = img.shape
    blank_image = img.copy()

    print("Found " + str(len(found_rect)) + " blocks in the frame")
    if len(found_rect) == 0:
        print("Could not find any blocks.")

    for point in intersections:
        cv2.circle(blank_image, (point.x, point.y), 5, (255, 255, 255), -1)

    rect_cnt = 0
    for index in found_rect:
        number_of_center += 1
        cv2.circle(blank_image, (int(index.center.x), int(index.center.y)), 7, (0, 255, 255), -1)
        cv2.putText(blank_image, str(rect_cnt + 1), (int(index.center.x + 5), int(index.center.y - 20)),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (50, 200, 200), 1)
        rect_cnt = rect_cnt + 1

    # for rect in found_rect:
    #     rect.drawDiagonal1(blank_image)
    #     rect.drawDiagonal2(blank_image)
    #
    # cv2.imshow("Removed Background", img_filtered)
    # cv2.imshow("Closing", closing)
    # cv2.imshow("Increast Contrast", contrast)
    # cv2.imshow("Canny Edges", edges)
    # cv2.imshow("Originally detected lines", unext_img)
    # cv2.imshow("Displaying Result", blank_image)
    # cv2.imshow("Extend the lines", ext_img)
    # cv2.waitKey()
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()

	return found_rect[0]