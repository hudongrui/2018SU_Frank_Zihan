from __future__ import division
import subprocess
import rospy
import numpy as np
import interceptHelper as iH
import graspObjectImageFunc as gi
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

##################################################################################
# Debug helper
# -1    -- enable all debugging feature
# 0     -- disable debug
# 1     -- matrix debugging
# 2     -- edge detection debug
# 3     -- grasp angle debug -- from Zihan: don't use it

debugMode = 0
##################################################################################

# this is the our desired Quaternion matrix
# TODO: try to figure out the Quaternion matrix mathematically
dQ = [-0.7218173645115756, -0.6913802266664445, 0.014645313419448478, 0.027542499143427157]

safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]

pre_grasp_pos = [-1.630677734375, -0.559880859375, -0.5919228515625, 0.723537109375, 0.4400439453125,
                 1.5005537109375,
                 1.35516796875]

moved_times = 0
square_list = 0

number_of_blocks_left = 100
result_block_list = []
block_index = 0

while square_list is not None and number_of_blocks_left != 0:
    # Pre-grasping joint angles
    rospy.sleep(1)

    Gp.move(limb, pre_grasp_pos, 0.3)
    square_list = None

    while square_list is None:
        img = Gp.take_picture(0, 30)
        if debugMode == 2 or debugMode == -1:
            cv2.imshow("Only the dots", img)
            cv2.waitKey()
            # objLoc, new_frame = Gp.detect_block(frame)
            # cv2.imshow("Image Captured", new_frame)
        square_list = iH.square_img_to_centers_list(img)
        number_of_blocks_left = len(square_list)

    if debugMode == 1 or debugMode == -1:
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(200, 200)
        print "w ", worldVec
        print "r ", rot
        print "d ", dQ
        drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=worldVec[0], p_y=worldVec[1], p_z=0.3,
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        Gp.move(limb, positions=drop_block_pos, move_speed=0.2)
        sys.exit()
        break
    else:
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())

    # Move above the desired block to generate better grasp model
    moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                     p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.443,
                                     # q_x=0, q_y=0, q_z=0, q_w=0)
                                     q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

    Gp.move(limb, positions=moveJoint, move_speed=0.2)

    # Retake image about the block for recognition
    frame = Gp.take_picture(0, 30)

    square_list = iH.square_img_to_centers_list(frame)

    # print("found square position: ", square_list[0].getCenterX(), square_list[0].getCenterY(), "\n")
    H, W, Ang = gi.predictGraspOnImage(frame, [square_list[0].getCenter().x, square_list[0].getCenter().y])
    # print("found the best H and W: ", H, W)

    if debugMode == 3:
        print("grasp master predict", Ang)
    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())
    Ang = square_list[0].getAngle()

    if debugMode == 3:
        print("my predict", Ang)
    Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1)
    rospy.sleep(1)

    if debugMode == 3:
        break

# TODO: change this so that placing the block is not hardcoded
    movingLoc = [0.83, 0 + 0.05, 0.02 + 0.04 * moved_times]
    drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                      p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

    Gp.move(limb, drop_block_pos, 0.2)

    block = square_list[0]
    block.setIndex(block_index)
    block.setLocation(movingLoc, dQ)
    result_block_list.append(block)

    rospy.sleep(1)
    gripper.open()
    rospy.sleep(0.5)

    # While loop stuff
    moved_times += 1
    block_index += 1
    number_of_blocks_left -= 1

    if debugMode == 3:
        Gp.move(limb, safe_move_r2l, 0.5)


