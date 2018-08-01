from __future__ import division

import copy
import sys

import cv2
import intera_interface
import intera_interface.head_display as head
import rospy

import GraspingHelperClass as Gp
import graspObjectImageFunc as gi
import interceptHelper as iH

import transformations as tfs
import numpy as np
import math

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
# 3     -- grasp angle debug
# 4     -- collision test w/o breaking the robot
# 5     -- Demo Mode
# 6     -- angle for placing robot

debugMode = 0
##################################################################################

# this is the our desired Quaternion matrix
# TODO: try to figure out the Quaternion matrix mathematically
dQ = [-0.7218173645115756, -0.6913802266664445, 0.014645313419448478, 0.027542499143427157]

safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]

# pre_grasp_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                      p_x=0.2, p_y=-0.7869, p_z=0.4432,
#                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
# pre_grasp_pos = [-2.00145703125, -0.7672353515625, 1.23169140625,
#                  0.8815908203125, -0.8251630859375, 1.866720703125, -4.08534765625]
pre_grasp_pos = [-1.630677734375, -0.559880859375, -0.5919228515625, 0.723537109375, 0.4400439453125, 1.5005537109375,
                 1.35516796875]

moved_times = 0
square_list = 0

number_of_blocks_left = 100
result_block_list = []
block_index = 0

# TODO: below returns a set of coordinates where we want to drop the block.
task = iH.drop_destinations()

if debugMode == 5:
    for drop_off_location in task:
        # Pre-grasping joint angles
        Gp.move(limb, pre_grasp_pos, 0.2)
        rospy.sleep(2)
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
            Gp.move(limb, positions=drop_block_pos, speed_ratio=0.2)
            sys.exit()
            break
        else:
            worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())

        # Move above the desired block to generate better grasp model
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1], p_z=0.443,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

        Gp.move(limb, positions=moveJoint, speed_ratio=0.2)

        # Retake image about the block for recognition

        img = Gp.take_picture(0, 30)
        square_list = iH.square_img_to_centers_list(img)

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        # print("found square position: ", square_to_find.getCenterX(), square_to_find.getCenterY(), "\n")
        H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
        # print("found the best H and W: ", H, W)

        # if debugMode == 3:
        #     print("grasp master predict", Ang)
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        print("Matrix" + str(hom_Mtrx_c_b[0]) + " " + str(hom_Mtrx_c_b[1]))
        Ang = square_to_find.getAngle(square_list)
        # if debugMode == 3:
        #     print("my predict", Ang)
        Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1)
        rospy.sleep(1)

        # if debugMode == 3:
        #     gripper.close()
        #     rospy.sleep(1)
        #     gripper.open()
        #     Gp.move(limb, pre_grasp_pos, 0.2)
        #     break

        movingLoc = drop_off_location
        pre_moving_loc = copy.deepcopy(drop_off_location)
        pre_moving_loc[2] += 0.3
        dQ = Gp.euler_to_quaternion(pre_moving_loc[3])
        pre_drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=pre_moving_loc[0], p_y=pre_moving_loc[1], p_z=pre_moving_loc[2],
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        Gp.move(limb, pre_drop_block_pos, 0.2)
        rospy.sleep(1)
        Gp.move(limb, drop_block_pos, 0.2)

        block = square_list[0]
        block.setIndex(block_index)
        block.setLocation(movingLoc, dQ)
        # result_block_list.append(block)

        rospy.sleep(1.5)
        gripper.open()
        rospy.sleep(1)

        Gp.move(limb, pre_drop_block_pos, 0.25)
        # While loop stuff
        moved_times += 1
        block_index += 1
        # number_of_blocks_left -= 1

else:
    while square_list is not None and number_of_blocks_left != 0 and debugMode != 4:
        # Pre-grasping joint angles
        Gp.move(limb, pre_grasp_pos, 0.3)
        rospy.sleep(1)
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
            Gp.move(limb, positions=drop_block_pos, speed_ratio=0.2)
            sys.exit()
            break
        else:
            worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())

        # Move above the desired block to generate better grasp model
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1], p_z=0.443,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

        Gp.move(limb, positions=moveJoint, speed_ratio=0.2)

        # Retake image about the block for recognition

        img = Gp.take_picture(0, 30)
        square_list = iH.square_img_to_centers_list(img)

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        print("found square position: ", square_to_find.getCenterX(), square_to_find.getCenterY(), "\n")
        H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
        print("found the best H and W: ", H, W)

        if debugMode == 3:
            print("grasp master predict", Ang)
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        print("Matrix" + str(hom_Mtrx_c_b[0]) + " " + str(hom_Mtrx_c_b[1]))
        Ang = square_to_find.getAngle(square_list)
        if debugMode == 3:
            print("my predict", Ang)
        Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1)
        rospy.sleep(1)

        if debugMode == 3:
            gripper.close()
            rospy.sleep(1)
            gripper.open()
            Gp.move(limb, pre_grasp_pos, 0.3)
            break

        # TODO: This allows the robot to vertically stack up the blocks
        movingLoc = [0.72, 0 + 0.045, 0.005 + 0.045 * moved_times]
        drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

        Gp.move(limb, drop_block_pos, 0.2)

        block = square_list[0]
        block.setIndex(block_index)
        block.setLocation(movingLoc, dQ)
        # result_block_list.append(block)

        rospy.sleep(1.5)
        gripper.open()
        rospy.sleep(1)

        movingLoc = [0.72, -0.8, 0.25 + 0.2 + 0.045 * moved_times]
        drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        Gp.move(limb, drop_block_pos, 0.3)
        # While loop stuff
        moved_times += 1
        block_index += 1
    # number_of_blocks_left -= 1

if debugMode != 3 and debugMode != 6:
    Gp.move(limb, safe_move_r2l, 0.3)
    rospy.sleep(1)

if debugMode == 4:
    still_safe_move = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                           p_x=0, p_y=-0.8, p_z=.1,
                                           q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
    Gp.move(limb, still_safe_move, 0.2)
    not_safe_move = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=0, p_y=-0.8, p_z=-.2,
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
    Gp.move(limb, not_safe_move, 0.01)

