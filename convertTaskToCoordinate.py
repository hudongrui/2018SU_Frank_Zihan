import cv2
import GraspingHelperClass as Gp
import interceptHelper as iH
from skimage import io
import numpy as np


# Read all the image file
task_img = []

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


	for i in range (len(cnts)):

		cnt = cnts[i]

		if cv2.contourArea(cnt) > 2000 and cv2.contourArea(cnt) < 5000 and hierarchy[0, i, 2] == -1:
			rect = cv2.minAreaRect(cnt)
			theta = 90.0 - abs(rect[2])

			box = cv2.boxPoints(rect)
			box = np.int0(box)
			# cv2.drawContours(blank_img,[box],0,(255,0,0),2)

			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			x = width/2 - cx
			y = height/2 - cy
			z = 0

			list_of_coordinates.append([x, y, z, theta])

	# cv2.imshow("Contour", blank_img)
	# cv2.waitKey()
	task_coordinate.append(list_of_coordinates)

return task_coordinate

def get_loc_by_pixel(workspace, previous_task_number):
	
    if workspace:
        # Workspace on human's left
        ctr_x, ctr_y, ctr_z = Gp.in_to_m(25 + 3.2), Gp.in_to_m(19), 0.1
    else:
        # Workspace on human's right
        ctr_x, ctr_y, ctr_z = Gp.in_to_m(25 + 3.2), Gp.in_to_m(-19), 0.1

    task = tasks[previous_task_number]

    locations = []

    factor = 0.035238

    for loc in task:	
        px, py, z = loc[0], loc[1], loc[2]

        x, y, z = ctr_x + px * factor, ctr_y + py * factor, ctr_z + z
        locations.append([x, y, z, theta])

    return locations, previous_task_number + 1




