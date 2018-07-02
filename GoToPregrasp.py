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
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

# Welcome slide
headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/hello.JPG")
rospy.sleep(1)
# Pre-grasping joint angles
pre_grasp_pos = [-1.630677734375, -0.559880859375, -0.5919228515625, 0.723537109375, 0.4400439453125, 1.5005537109375,
				 1.35516796875]
# print limb.joint_angles()
# Gp.move(limb, safe_move_r2l, 0.2)

rospy.sleep(1)

# Gp.move(limb, pre_grasp_pos, 0.2)


# frame = Gp.take_picture(0, 30)  # changed to port 1 cry 15/02/2018

# objLoc = Gp.detect_block(8, frame)

# worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

# moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                      p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,
#                                      # q_x=0, q_y=0, q_z=0, q_w=0)
#                                      q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])
# Gp.move(limb, positions=moveJoint, move_speed=0.2)
    