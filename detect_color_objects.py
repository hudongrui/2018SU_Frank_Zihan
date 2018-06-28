import cv2
import numpy as np
import datetime


def detect_objects(fruit_number):
    # take picture from webcam
    now = datetime.datetime.now()
    file_name = now.strftime("%Y-%m-%d")
    img = cv2.imread(str(take_picture(file_name)), 1)
    # img = cv2.imread("/home/Documents/2018-01-27.png", 1)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_range_apple = np.array([10, 150, 70], dtype=np.uint8)
    upper_range_apple = np.array([179, 255, 100], dtype=np.uint8)
    red = [30, 30, 200]

    lower_range_avocado = np.array([10, 10, 5], dtype=np.uint8)
    upper_range_avocado = np.array([50, 90, 110], dtype=np.uint8)
    red2 = [10, 10, 255]

    lower_range_banana = np.array([20, 100, 150], dtype=np.uint8)
    upper_range_banana = np.array([40, 200, 255], dtype=np.uint8)
    yellow = [0, 240, 255]

    lower_range_grape = np.array([20, 50, 0], dtype=np.uint8)
    upper_range_grape = np.array([179, 200, 50], dtype=np.uint8)
    purple = [255, 127, 255]

    lower_range_orange = np.array([0, 160, 125], dtype=np.uint8)
    upper_range_orange = np.array([100, 255, 150], dtype=np.uint8)
    orange1 = [50, 127, 255]

    lower_range_pear = np.array([40, 70, 100], dtype=np.uint8)
    upper_range_pear = np.array([50, 200, 255], dtype=np.uint8)
    green = [74, 111, 55]

    lower_range_peach = np.array([0, 100, 140], dtype=np.uint8)
    upper_range_peach = np.array([150, 150, 170], dtype=np.uint8)
    pink = [74, 111, 55]

    def apple():
        center_of_apple = find_element_within_range(img, imgHSV, lower_range_apple, upper_range_apple, red)
        print "You asked for an apple.\n"
        return center_of_apple

    def avocado():
        center_of_avocado = find_element_within_range(img, imgHSV, lower_range_avocado, upper_range_avocado, red2)
        print "You asked for an avocado.\n"
        return center_of_avocado

    def banana():
        center_of_banana = find_element_within_range(img, imgHSV, lower_range_banana, upper_range_banana, yellow)
        print "You asked for a banana.\n"
        return center_of_banana

    def grape():
        center_of_grape = find_element_within_range(img, imgHSV, lower_range_grape, upper_range_grape, purple)
        print "You asked for a grape.\n"
        return center_of_grape

    def orange():
        center_of_orange = find_element_within_range(img, imgHSV, lower_range_orange, upper_range_orange, orange1)
        print "You asked for an orange.\n"
        return center_of_orange

    def pear():
        center_of_pear = find_element_within_range(img, imgHSV, lower_range_pear, upper_range_pear, green)
        print "You asked for a pear.\n"
        return center_of_pear

    def peach():
        center_of_peach = find_element_within_range(img, imgHSV, lower_range_peach, upper_range_peach, pink)
        print "You asked for a peach.\n"
        return center_of_peach

    # map the inputs to the function blocks
    fruits = {1: apple, 2: avocado, 3: banana, 4: grape, 5: orange, 6: pear, 7: peach,}
    desired_fruit_location = fruits[fruit_number]()

    cv2.imshow('image', img)
    cv2.waitKey(0)

    cv2.destroyAllWindows()

    return desired_fruit_location


def find_element_within_range(image, imgHSV, lower_range, upper_range, color):
    mask = cv2.inRange(imgHSV, lower_range, upper_range)

    cv2.imshow('mask', mask)
    cv2.waitKey(0)

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    mask = cv2.erode(mask, element, iterations=2)
    mask = cv2.dilate(mask, element, iterations=2)
    mask = cv2.erode(mask, element)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maximumArea = 0
    bestContour = None
    for contour in contours:
        currentArea = cv2.contourArea(contour)
        if currentArea > maximumArea:
            bestContour = contour
            maximumArea = currentArea
    # Create a bounding box around the biggest red object
    x, y, w, h = (0, 0, 0, 0)

    if bestContour is not None:
        x, y, w, h = cv2.boundingRect(bestContour)
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)

    if x != 0:
        cv2.circle(image, (x + w / 2, y + h / 2), 3, 3)
        center = (x + w / 2, y + h / 2)
    else:
        center = 0

    return center


def take_picture(file_name):
    # Camera 0 is the camera on the arm
    camera_port = 0

    # Number of frames to throw away while the camera adjusts to light levels
    ramp_frames = 30

    # Now we can initialize the camera capture object with the cv2.VideoCapture class.
    # All it needs is the index to a camera port.
    camera = cv2.VideoCapture(camera_port)

    # Captures a single image from the camera and returns it in PIL format
    def get_image():
        # read is the easiest way to get a full image out of a VideoCapture object.
        retval, im = camera.read()
        return im

    # Ramp the camera - these frames will be discarded and are only used to allow v4l2
    # to adjust light levels, if necessary
    for i in xrange(ramp_frames):
        temp = get_image()
    print("Taking image...")
    # Take the actual image we want to keep
    camera_capture = get_image()
    print("Done")
    path_to_file = "/home/team18/image-rec-color/" + str(file_name) + ".png"
    print path_to_file
    # A nice feature of the imwrite method is that it will automatically choose the
    # correct format based on the file extension you provide. Convenient!
    cv2.imwrite(path_to_file, camera_capture)

    # # You'll want to release the camera, otherwise you won't be able to create a new
    # # capture object until your script exits
    camera.release()
    return path_to_file

    # detect_objects(1)
