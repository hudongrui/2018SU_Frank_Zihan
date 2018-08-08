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

rospy.init_node("FrankZihanSum")
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
# 5     -- Previous demo Mode
# 6     -- angle for placing robot
# 7     -- trajectory planning
debugMode = 0
##################################################################################
DemoMode = 0
##################################################################################
# ~~~~~~~~~~~~~~~~~~ girigiri ai~~~~~~~~~~~~~~~
crazyMode = False
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

test_location = [-0.05053515625, 0.144181640625, -1.3031396484375, -0.9186611328125, -1.79606640625,
                 -1.713986328125, -1.4706103515625]

collision_move = [-0.48626953125, 0.2361240234375, 1.8887138671875, -1.71039453125, -1.2699970703125,
                  1.2547158203125, -3.5353935546875]

###############################################################
# added by Zihan 08/02/2018

moveComm = Gp.moveit_commander
moveComm.roscpp_initialize(sys.argv)

robot = moveComm.RobotCommander()
scene = moveComm.PlanningSceneInterface()

group_name = "right_arm"
group = moveComm.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               Gp.moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
# print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
# print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print "============ Robot Groups:", robot.get_group_names()

# link_names = robot.get_link_names()
# print "============ Robot Links:", robot.get_link_names()
# TODO: figure out why i cannot get current joint values
# joint_goal = group.get_current_joint_values()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
# print "============ Printing robot state"
# print robot.get_current_state()
# print ""

Gp.load_objects(scene, planning_frame)
Gp.load_camera_w_mount(scene, robot, planning_frame)
###############################################################

moved_times = 0
square_list = 0

number_of_blocks_left = 100
result_block_list = []
block_index = 0

# TODO: below returns a set of coordinates where we want to drop the block.
task = iH.drop_destinations()

# enableExecutionDurationMonitoring(false)
if debugMode == 7:
    rospy.sleep(1)
    Gp.avoid_move(group, safe_move_r2l)
    rospy.sleep(1)
    Gp.avoid_move(group, collision_move)
    rospy.sleep(1)
    Gp.avoid_move(group, safe_move_r2l)
    # rospy.sleep(1)
    # Gp.move_improved(limb, group, pre_grasp_pos)
elif debugMode == 5:
    for drop_off_location in task:
        Gp.smooth_move(limb, pre_grasp_pos)
        # Pre-grasping joint angles
        move_speed = 0.5
        if crazyMode is True:
            move_speed = 1.0
        Gp.avoid_move(group, pre_grasp_pos)
        rospy.sleep(1)
        square_list = None

        timeout = 0
        while square_list is None:
            if timeout > 5:
                rospy.logerr("No block exists in the frame. Returning to initial position")
                break
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
            Gp.avoid_move(group, positions=drop_block_pos)
            sys.exit()
            break
        else:
            worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())

        # Move above the desired block to generate better grasp model
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1], p_z=0.443,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

        Gp.smooth_move(limb, positions=moveJoint)

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
        #     Gp.move_improved(group, pre_grasp_pos, 0.5)
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
        Gp.smooth_move(limb, pre_drop_block_pos)
        rospy.sleep(1)
        Gp.smooth_move(limb, drop_block_pos)

        block = square_list[0]
        block.setIndex(block_index)
        block.setLocation(movingLoc, dQ)
        # result_block_list.append(block)

        rospy.sleep(1)
        gripper.open()
        rospy.sleep(1)

        Gp.smooth_move(limb, pre_drop_block_pos)
        # While loop stuff
        moved_times += 1
        block_index += 1
        # number_of_blocks_left -= 1

elif debugMode != 7:
    try:
        while square_list is not None and number_of_blocks_left != 0 and debugMode != 4:
            Gp.smooth_move(limb, pre_grasp_pos)
            # Pre-grasping joint angles
            move_speed = 0.5
            if crazyMode is True:
                move_speed = 1.0
            Gp.avoid_move(group, pre_grasp_pos, move_speed)
            rospy.sleep(1)
            square_list = None

            timeout = 0
            while square_list is None:
                if timeout > 5:
                    rospy.logerr("No block exists in the frame. Returning to initial position")
                    break
                img = Gp.take_picture(0, 30)
                if debugMode == 2 or debugMode == -1:
                    cv2.imshow("Only the dots", img)
                    cv2.waitKey()
                    # objLoc, new_frame = Gp.detect_block(frame)
                    # cv2.imshow("Image Captured", new_frame)
                square_list = iH.square_img_to_centers_list(img)
                timeout += 1

            try:
                number_of_blocks_left = len(square_list)
            except TypeError:
                break

            if debugMode == 1 or debugMode == -1:
                worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(200, 200)
                print "w ", worldVec
                print "r ", rot
                print "d ", dQ
                drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                                      p_x=worldVec[0], p_y=worldVec[1], p_z=0.3,
                                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
                Gp.avoid_move(group, positions=drop_block_pos, speed_ratio=0.5)
                sys.exit()
                break
            else:
                worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_list[0].getCenterX(), square_list[0].getCenterY())

            # Move above the desired block to generate better grasp model
            moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                             p_x=worldVec[0], p_y=worldVec[1], p_z=0.443,
                                             # q_x=0, q_y=0, q_z=0, q_w=0)
                                             q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

            Gp.smooth_move(limb, positions=moveJoint)

            # Retake image about the block for recognition

            img = Gp.take_picture(0, 30)
            square_list = iH.square_img_to_centers_list(img)

            square_to_find = iH.find_square_closest_to_center(img, square_list)

            rospy.logdebug("found square position: ", square_to_find.getCenterX(), square_to_find.getCenterY(), "\n")
            H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
            rospy.logdebug("found the best H and W: ", H, W)

            worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
            print("Matrix" + str(hom_Mtrx_c_b[0]) + " " + str(hom_Mtrx_c_b[1]))
            Ang = square_to_find.getAngle(square_list)
            Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1)
            rospy.sleep(1)

            if debugMode == 3:
                gripper.close()
                rospy.sleep(1)
                gripper.open()
                Gp.avoid_move(group, pre_grasp_pos, 0.3)
                break

            # TODO: This allows the robot to vertically stack up the blocks
            release_position = [0.72, 0 + 0.045, 0.005 + 0.045 * moved_times]
            pre_release = [0.72, -0.8, 0.25 + 0.2 + 0.045 * moved_times]
            drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                                  p_x=release_position[0], p_y=release_position[1], p_z=release_position[2],
                                                  q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
            pre_drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                                      p_x=pre_release[0], p_y=pre_release[1], p_z=pre_release[2],
                                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
            Gp.avoid_move(group, pre_drop_block_pos)
            Gp.smooth_move(limb, drop_block_pos)

            block = square_list[0]
            block.setIndex(block_index)
            block.setLocation(drop_block_pos, dQ)
            # result_block_list.append(block)

            rospy.sleep(1.5)
            gripper.open()
            rospy.sleep(1)

            Gp.smooth_move(limb, drop_block_pos)
            # While loop stuff
            moved_times += 1
            block_index += 1
    # number_of_blocks_left -= 1
    finally:
        if debugMode != 3 and debugMode != 6:
            move_speed = 0.5
            if crazyMode is True:
                move_speed = 1.0
            Gp.smooth_move(limb, safe_move_r2l)
            rospy.sleep(1)

if debugMode == 4:
    still_safe_move = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                           p_x=0, p_y=-0.8, p_z=.1,
                                           q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
    Gp.avoid_move(group, still_safe_move, 0.5)
    not_safe_move = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=0, p_y=-0.8, p_z=-.2,
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
    Gp.avoid_move(group, not_safe_move, 0.01)

# required ending chuck -- starting
# Gp.remove_objects(scene)
# Gp.remove_camera_w_mount(scene)
moveComm.roscpp_shutdown()
# required ending chuck -- ending

