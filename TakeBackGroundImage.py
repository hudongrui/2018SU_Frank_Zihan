import cv2
import GraspingHelperClass as Gp

# frame = Gp.take_picture(0, 30) # Use this function for webcam
frame = Gp.get_video() # Use this function for Kinect
cv2.imshow("Displaying Image", frame)
cv2.waitKey()
cv2.destroyAllWindows()
cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/Background_Left_Kinect.jpg", frame)