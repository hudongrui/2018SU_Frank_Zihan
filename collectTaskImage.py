import cv2
import GraspingHelperClass as Gp
import interceptHelper as iH
from skimage import io
import numpy as np


img = Gp.take_picture(0, 30)

mask = io.imread("Background_Right.jpg")

img_filtered = cv2.subtract(mask, img)

img_filtered = iH.rm_shadow(img_filtered)

kernel = np.ones((5, 5), np.uint8)
closing = cv2.morphologyEx(img_filtered, cv2.MORPH_CLOSE, kernel)

contrast = iH.increase_contrast(closing)

greyscale_img = cv2.cvtColor(contrast, cv2.COLOR_BGR2GRAY)
# greyscale_img = np.invert(greyscale_img)
retval, greyscale_img = cv2.threshold(greyscale_img, 50, 255, cv2.THRESH_BINARY)

cnt_img, cnts, hierarchy = cv2.findContours(greyscale_img, cv2.RETR_LIST,
                        cv2.CHAIN_APPROX_SIMPLE)

	

height, width, _ = img.shape

blank_img = np.zeros((height, width, 3), np.uint8)
blank_img[:,:] = (255,255,255) 


for cnt in cnts:
	if cv2.contourArea(cnt) > 2000:
		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		cv2.drawContours(blank_img,[box],0,(255,0,0),2)
# cv2.drawContours(blank_img, cnts,-1,(255,0,0),2)

cv2.imshow("Saving Image", blank_img)
cv2.waitKey()

print 'Which task is this?'

key_pressed = input("Enter a number to save this image: ")

cv2.destroyAllWindows()
cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/task_" + str(key_pressed) + ".jpg", blank_img)


# print 'press s to save picture and exit'
# print 'press c to save picture and continue'
# print 'press q to exit'

# k = cv2.waitKey(5) & 0xFF
# if k == ord('s'):
# 	cv2.destroyAllWindows()
# 	cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/task_" + str(key_pressed) + ".jpg", blank_img)
# 	break
# elif k == ord('c'):
# 	cv2.destroyAllWindows()
# 	cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/task_" + str(key_pressed) + ".jpg", blank_img)
# elif k == ord('q'):
# 	cv2.destroyAllWindows()
# 	break
# else:
# 	cv2.destroyAllWindows()
# 	print 'Please retake image'

