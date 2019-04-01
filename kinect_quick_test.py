import frame_convert2
import numpy as np
import freenect
import cv2
from skimage import io


# img = np.array(frame_convert2.video_cv(freenect.sync_get_video()[0]))
img = io.imread("Background_Right.jpg")
cv2.imshow("",img)
cv2.waitKey()