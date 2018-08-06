# Import the modules
import cv2
import numpy as np
import argparse as ap

# Make sure to run this code only once for a single image, running it multiple times will shrink the image smaller and smaller
parser = ap.ArgumentParser()
parser.add_argument("-i", "--image", help="Path to Image", required="True")
args = vars(parser.parse_args())

# Read the input image
im = cv2.imread(args["image"])

scale_percent = 20  # Percent of original size
width = int(im.shape[1] * scale_percent / 100)
height = int(im.shape[0] * scale_percent / 100)
dim = (width, height)

# resize image
resized = cv2.resize(im, dim, interpolation=cv2.INTER_AREA)

print('Resized Dimensions : ', resized.shape)

cv2.imshow("Resized image", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()

dir = args["image"]

cv2.imwrite(dir, resized)