from __future__ import print_function
from __future__ import division
from sympy.solvers import solve
from sympy import Symbol
import math
import warnings
import cv2
import numpy as np
import time
import copy


def refine_range(angle, lowerLimit=None, upperLimit=None, modifier=None):
    # method used for changing angle into range, could be used in other situation
    if lowerLimit is None:
        lowerLimit = -90
    if upperLimit is None:
        upperLimit = 90
    if modifier is None:
        modifier = 180

    print("lower is ", lowerLimit)
    print("upper is ", upperLimit)
    print("mod is ", modifier)

    if angle < lowerLimit:
        angle += modifier
    elif angle > upperLimit:
        angle -= modifier
    return angle

print(refine_range(160))
print(refine_range(160, -80, 80, 160))
