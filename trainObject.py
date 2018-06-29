import subprocess
# import roslib
# roslib.load_manifest('your_package_name')
import rospy
import graspObjectImageFunc as gi
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global GRIPPER
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

# Welcome slide
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/hello.JPG")
rospy.sleep(1)

# Reset joint angles
safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]

# Gesture joint angles
gest_position = [-0.3003134765625, -1.312572265625, 0.371109375, 0.4870419921875, 0.021302734375, 1.1612021484375,
                 1.62130078125]

# Pre-grasping joint angles
pre_grasp_pos = [-1.630677734375, -0.559880859375, -0.5919228515625, 0.723537109375, 0.4400439453125, 1.5005537109375,
				 1.35516796875]
# print limb.joint_angles()
# Gp.move(limb, safe_move_r2l, 0.2)

rospy.sleep(1)

Gp.move(limb, pre_grasp_pos, 0.2)

# frame = Gp.take_picture(0, 30)  # changed to port 1 cry 15/02/2018

# objLoc = Gp.detect_block(8, frame)

# worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

# moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                      p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,
#                                      # q_x=0, q_y=0, q_z=0, q_w=0)
#                                      q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

# Gp.move(limb, positions=moveJoint, move_speed=0.2)

frame = Gp.take_picture(0, 30)

cv2.imshow("Image Captured", frame)
# cv2.waitKey()
# cv2.destroyAllWindows()

objLoc = Gp.detect_block(8, frame)

worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

H, W, Ang = gi.predictGraspOnImage(frame, objLoc)

Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1)
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