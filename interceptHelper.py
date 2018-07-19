from __future__ import division
from sympy.solvers import solve
from sympy import Symbol
import math
import warnings
import cv2
import numpy as np
import time

##################################################################################
# debugMode helper
# -1    -- enable all debugging feature
# 0     -- disable debug
# 1     -- matrix debugging
# 2     -- edge detection debug

debugMode = 2
##################################################################################

#########################################################
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
    b1 = pt1[1] - pt1[0] * m1

    m2 = (pt4[1] - pt3[1]) / (pt4[0] - pt3[0])
    b2 = pt3[1] - pt3[0] * m2

    # Ignore warning when getting a infinity slope
    warnings.filterwarnings("ignore")

    # Consider if the lines are horizontal or vertical to cause a non-resolvable slope for intersection
    if m1 == m2:
        # print("Same Slope")
        return None, None, None, False
    elif abs(m1) == float('Inf') and abs(m2) <= 0.1:
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

    # Solve for intersection
    x = Symbol('x')
    solution = solve((m1 - m2) * x + b1 - b2, x)
    if len(solution) != 1:
        # print("Identical Lines")
        return None, None, None, False

    # Check if intersects fall in the range of two lines
    elif pt1[0] <= solution <= pt2[0] and pt3[0] <= solution <= pt4[0]:
        # print("Solution is " + str(float(solution[0])))

        x_intersect = int(solution[0])
        y_intersect = int(m2 * solution[0] + b2)

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


def extend_line(line):
    x1, y1, x2, y2 = line[0]
    length = int(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    # TODO: Adjust the following threshold to pass the lines
    one_block_len = 65
    ratio = float(abs(1.6 * (one_block_len - length) / length))
    if 2 * one_block_len <= length <= 2.5 * one_block_len:
        # print("Two Blocks")
        return line
    elif 0.8 * one_block_len < length < 1.2 * one_block_len:
        # print("One Block")
        ratio = float(abs(5 * (one_block_len - length) / length))
    elif length < 0.8 * one_block_len:
        ratio = float(abs((1.6 * one_block_len - length) / length))

        # TODO: Extends lines based on its length, might need change ratio
        # ratio = 0.6
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
    # # print("Ratio is: " + str(ratio))
    # print("Extended Length is: " + str(Extended_Length))
    return [extended]
    # return line


def increase_contrast(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    cv2.imshow("lab", lab)

    # -----Splitting the LAB image to different channels-------------------------
    l, a, b = cv2.split(lab)
    cv2.imshow('l_channel', l)
    cv2.imshow('a_channel', a)
    cv2.imshow('b_channel', b)

    # -----Applying CLAHE to L-channel-------------------------------------------
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    cv2.imshow('CLAHE output', cl)

    # -----Merge the CLAHE enhanced L-channel with the a and b channel-----------
    limg = cv2.merge((cl, a, b))
    cv2.imshow('limg', limg)

    # -----Converting image from LAB Color model to RGB model--------------------
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    return final


def rm_nearby_intersect(intersections):
    if len(intersections) != 0:
        i = 0
        for point_1 in intersections:
            j = 0
            for point_2 in intersections:
                if i < j:
                    x1, y1 = point_1.x, point_1.y
                    x2, y2 = point_2.x, point_2.y
                    if abs(x1 - x2) <= 15 and abs(y1 - y2) <= 15:
                        intersections.remove(point_2)
                j = j + 1
            i = i + 1
    return intersections


# def adjust_gamma(image, gamma=1.0):
#     # build a lookup table mapping the pixel values [0, 255] to
#     # their adjusted gamma values
#     invGamma = 1.0 / gamma
#     table = np.array([((i / 255.0) ** invGamma) * 255
#                       for i in np.arange(0, 256)]).astype("uint8")
#
#     # apply gamma correction using the lookup table
#     return cv2.LUT(image, table)

def rm_duplicates(rects, intersections):

    def is_near(ctr, intersects):
        bol = False
        for index in intersects:
            # print(str(abs(ctr.x - index.y)) + " and " + str)
            if abs(ctr.x - index.x) <= 25 and abs(ctr.y - index.y) <= 25:
                bol = True
                break
        return bol

    # for rect in rects:
    #     rect_center = rect.center
    #     if is_near(rect_center, intersections):
    #         rects.remove(rect)

    centers = []
    for rect in rects:
        rect_center = rect.center
        if not is_near(rect_center, centers):
            if not is_near(rect_center, intersections):
                centers.append(rect_center)
    return centers


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
                    image[y, x] = [0,0,0]

    return image


# This method is currently unused, substituted by cv2.subtract
def rm_background(img, mask):
    h, w, _ = img.shape
    # print(str(img[h//2, w//2]))
    for y in range(h):
        for x in range(w):
            p_img = img[y, x]
            p_mask = mask[y, x]
            r, g, b = [abs(i - j) for i, j in zip(p_img, p_mask)]
            t = 40
            if r < t and g < t and b < t:
                img[y, x] = [0, 0, 0]

    return img


def rm_false_positive(rect_centers, img):
    img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_RGB2GRAY)
    centers = []
    for rect in rect_centers:
        # print(img_gray[int(rect.y), int(rect.x)])
        # print("at (" + str(int(rect.y)) + ", " + str(int(rect.x)) + ")")
        if np.any(img_gray[int(rect.y), int(rect.x)] != 0):
            # print("False Positive at (" + str(int(rect.x)) + ", " + str(int(rect.y)) + ")")
            centers.append(rect)

    return centers


class Intersect:
    def __init__(self, x_intersect, y_intersect, theta=None, category=None):
        self.x = x_intersect
        self.y = y_intersect
        if theta is not None:
            self.theta = theta
        if category is not None:
            self.category = category


class Line:
    def __init__(self, start_point, end_point):
        self.start = start_point
        self.end = end_point
        self.theta = math.atan2((start_point.y - end_point.y), (start_point.x - end_point.x))
        self.length = math.hypot((start_point.x - end_point.x), (start_point.y - end_point.y))


def is_in_range_of_a_circle(point1, point2, radius_threshold=None):
    if radius_threshold is None:
        radius_threshold = 15
    return distance_between_points(point1, point2) < radius_threshold


def distance_between_points(point1, point2):
    return math.hypot((point2.x - point1.x), (point2.y - point1.y))


def categorize_rect(intersections):
    start_time = time.time()
    tmp_center_list = []
    list_of_squares = []
    tmp_intersection = intersections
    for starting_point in tmp_intersection:
        for next_point in tmp_intersection:
            # TODO: fix this bug -- Zihan
            # if len(tmp_center_list) > 2:
            #     standard_length = tmp_center_list[math.trunc(len(tmp_center_list) / 2) - 1].length
            #     if distance_between_points(starting_point, next_point) > standard_length * 1.5 or \
            #             distance_between_points(starting_point, next_point) < standard_length / 1.5:
            #         break
            if starting_point != next_point:
                base_line = Line(starting_point, next_point)
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
                    if is_in_range_of_a_circle(possible_1, third_point):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_1_c, forth_point):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
                    if is_in_range_of_a_circle(possible_2, third_point):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_2_c, forth_point):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
                    if is_in_range_of_a_circle(possible_3, third_point):
                        for forth_point in tmp_intersection:
                            if is_in_range_of_a_circle(possible_3_c, forth_point):
                                append_rec_list(list_of_squares,
                                                Rectangle(starting_point, next_point, third_point, forth_point),
                                                tmp_center_list)
    elapsed_time = time.time() - start_time
    print("the time elapsed for categorizing square is " + str(elapsed_time))
    return list_of_squares


# TODO: fix this bug -- Zihan
def append_rec_list(output_list, rect, center_list):
    output_list.append(rect)
    center_list.append(rect.center)


def mid_point(point1, point2):
    return Intersect((point1.x + point2.x) / 2, (point1.y + point2.y) / 2)


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
            self.distance = distance_between_points(point1, point3)
        else:
            self.distance = distance_between_points(point1, point3)
        if point4 is None:
            self.center = self.find_its_center_3()
        else:
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


def square_img_to_centers_list(img):
    img_shadowless = rm_shadow(img)
    kernel = np.ones((5, 5), np.uint8)
    img_erosion = cv2.erode(img_shadowless, kernel, iterations=1)
    img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)
    img_blurred_bilateral = cv2.bilateralFilter(img_dilation, 20, 50, 50)
    edges = cv2.Canny(img_blurred_bilateral, 200, 300)
    # cv2.imshow("edges", edges)
    # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=20, maxLineGap=60)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=32, minLineLength=30, maxLineGap=40)
    ext_lines = []
    for line in lines.copy():
        new_line = extend_line(line)
        ext_lines.append(new_line)
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
    intersections = rm_nearby_intersect(intersections)
    found_rect = categorize_rect(intersections)
    found_rect_centers = rm_duplicates(found_rect)

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
        return None
    if debugMode == 2 or debugMode == -1:
        cv2.imshow("Only the dots", blank_image)
        cv2.waitKey()
    return found_rect_centers
