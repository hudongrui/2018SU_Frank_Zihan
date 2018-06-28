import subprocess
import rospy
import graspObjectImageFunc as gi
import intera_interface
import intera_interface.head_display as head
import cv2

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global GRIPPER
gripper = intera_interface.Gripper('right')
gripper.open()
headDisplay = head.HeadDisplay()

# Welcome slide
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/hello.JPG")
rospy.sleep(1)

# silence please!
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/silence_please_preparing_to_record.JPG")
rospy.sleep(0.5)
subprocess.call('python ./NLP_class.py', shell=True)

# ======================================================================================
import GraspingHelperClass as Gp
import GestRecB

safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]
gest_position = [-0.3003134765625, -1.312572265625, 0.371109375, 0.4870419921875, 0.021302734375, 1.1612021484375,
                 1.62130078125]
print limb.joint_angles()
Gp.move(limb, safe_move_r2l, 0.2)

# print "Types of fruits: 1, apple; 2, avocado; 3, banana; 5, orange; 7, peach"
# type_of_fruit = raw_input("Enter the fruit you want: ")
with open("finalvalue.txt", "rb") as f:
   type_of_fruit = int(f.read())

# GESTURE RECOGNITION
#
if type_of_fruit == 1:
    # I heard apple
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_apple.JPG")
elif type_of_fruit == 2:
    # I heard avocado
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_avocado.JPG")
elif type_of_fruit == 3:
    # I heard banana
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_banana.JPG")
elif type_of_fruit == 5:
    # I heard orange
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_orange.JPG")
else:
    # I heard peach
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_peach.JPG")

# type_of_fruit = 1

inputDir = "examples/Sawyer_GesRec/input_dir"
outputDir = "examples/Sawyer_GesRec/output_dir"
Gp.move(limb, gest_position, 0.2)
ges = GestRecB.GestureRecognition()

rospy.sleep(1.5)
# # Which table?
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/where_is_it.JPG")
rospy.sleep(1)

# Point in...

array_for_looping = ["/home/team18/Grasp-Detector-master/sawyer_head/loop_these/1_point_in.JPG",
                      "/home/team18/Grasp-Detector-master/sawyer_head/loop_these/2_count_3.JPG",
                      "/home/team18/Grasp-Detector-master/sawyer_head/loop_these/3_count_2.JPG",
                      "/home/team18/Grasp-Detector-master/sawyer_head/loop_these/4_count_1.JPG",
                      "/home/team18/Grasp-Detector-master/sawyer_head/loop_these/5_point_because_picture_being_taken.JPG"]
headDisplay.display_image(array_for_looping)
#
side_of_user = ges.execute(inputDir, outputDir, headDisplay)
# side_of_user = 1
table = str(side_of_user)
if side_of_user == 1:
    # left side of user
    pre_grasp_pos = [-1.7220224609375, -1.1679736328125, -0.138830078125, 1.015201171875, 0.00028515625,
                     1.7164111328125,
                     -1.8061787109375]
    # I saw you point left
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_pointed_left.JPG")
else:
    # right side of user
    pre_grasp_pos = [0.8807666015625, -1.1366826171875, 0.9580830078125, 0.99812890625, -0.3362646484375,
                     1.837322265625,
                     -1.4236279296875]
    # I saw you point right
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_pointed_right.JPG")
    # Gp.move(limb, safe_move_r2l, 0.2)



Gp.move(limb, pre_grasp_pos, 0.2)

#
# GESTURE RECOGNITION
#

frame = Gp.take_picture(1, 30)  # changed to port 1 cry 15/02/2018

objLoc = Gp.detect_objects(type_of_fruit, frame)

# print objLoc

worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

# print worldVec
# TODO: change values on offset

if table == "1":
    moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                     p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,
                                     # q_x=0, q_y=0, q_z=0, q_w=0)
                                     q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])
else:
    moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                     p_x=worldVec[0], p_y=worldVec[1] - 0.07, p_z=0.5,
                                     # q_x=0, q_y=0, q_z=0, q_w=0)
                                     q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

# moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                  p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,  # q_x=0, q_y=0, q_z=0, q_w=0)
#                                  q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

Gp.move(limb, positions=moveJoint, move_speed=0.2)

frame = Gp.take_picture(1, 30)

# gscale = 0.234375
# imsize = max(frame.shape[:2])
# gsize = int(gscale * imsize)  # Size of grasp patch
#
# print "image shape is:", frame.shape

# cv2.imshow('image', frame)
# cv2.waitKey(0)

objLoc = Gp.detect_objects(type_of_fruit, frame)

# try:
#     H, W, Ang = gi.predictGraspOnImage(frame, objLoc)
# except ValueError:
#     print('failed 1st grasping attempt')

# frame = Gp.take_picture(0, 30)
# cv2.imshow('image', frame)
# cv2.waitKey(0)

H, W, Ang = gi.predictGraspOnImage(frame, objLoc)
# H = (H+worldVec[0])*0.5
# W = (W+worldVec[1])*0.5

# print "W is: ", W
# print "H is: ", H
# print "worldVec[0] is: ", worldVec[0]
# print "worldVec[1] is: ", worldVec[1]

Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], table)
rospy.sleep(1)
rospy.sleep(1)
drop_fruit_pos = [-0.402546875, -1.1622041015625, 0.3266787109375, 2.2412666015625, -0.301185546875,
                  0.469794921875,
                  -1.2894443359375]
Gp.move(limb, drop_fruit_pos, 0.2)
rospy.sleep(1)
gripper.open()
rospy.sleep(0.5)
Gp.move(limb, safe_move_r2l, 0.2)
rospy.sleep(0.5)

# Done
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/all_done.JPG")
rospy.sleep(1)
image_gest = cv2.imread("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg", 1)
resize_img = cv2.resize(image_gest, (1024, 600))
cv2.imwrite("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg", resize_img)
headDisplay.display_image("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg")
