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

debugMode = 100

###  Establish ROS connection and initialize robot
rospy.init_node("FrankZihanSu")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

##################################################################################
# Ignore this
crazyMode = False
##################################################################################
dQ = Gp.euler_to_quaternion(z=3.1415/2)
operation_height = 0.25

# Pre-grasping joint angles, default positions are camera centers
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=0.28, p_y=0.780, p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
gripper_center_human_right = [-0.0431328125, -0.4307822265625, 1.257787109375, 1.0017109375, -1.04582421875,
                              1.554017578125, 2.63585546875]
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=-0.15, p_y=0.780, p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
gripper_center_human_left = [-1.1013798828125, -0.66011328125, 1.33263671875, 1.671609375, -0.9153974609375,
                             1.3138095703125, 2.148287109375]
# this is the our desired Quaternion matrix
safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]
test_location = [1.56305078125, 0.09955078125, 2.441650390625, -1.2888828125, -1.8209716796875,
                 0.7058720703125, -0.894408203125]
collision_move = [-1.949134765625, 0.5128837890625, -3.0382353515625, 1.2715830078125, 2.7810556640625,
                  1.3979755859375, 1.5122421875]
home_position = [0,0,0,0,0,0,0 + 10/180*3.1415]
# Right before any grasping tasks, the robot would default its grasping
# workspace to be on the left side of the table, and move to a position that
# camera's focus is pointing down right at the center of the left table.
pre_grasp_pos = camera_center_human_right
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

planning_frame = group.get_planning_frame()

eef_link = group.get_end_effector_link()
Gp.load_objects(scene, planning_frame)
Gp.load_camera_w_mount(scene)
###############################################################
if debugMode == 18:
  exit()
moved_times = 0
square_list = 0

number_of_blocks_left = 0
block_index = 0

# false indicates pickup location is on the left and true is right
temp_workspace = True
iterations = 2  # how many times does the robot have to repeatedly grab the blocks
if debugMode == 0:
    Gp.move_move(limb, group, safe_move_r2l)

while iterations != 0:
    # Below returns a set of coordinates where we want to drop the block.
    task = iH.drop_destinations(temp_workspace)
    if temp_workspace:
        pre_grasp_pos = camera_center_human_right
    else:
        pre_grasp_pos = camera_center_human_left
    # Switch workspace for the next time
    temp_workspace = not temp_workspace

    for drop_off_location in task:
        # Pre-grasping joint angles
        move_speed = 0.5
        if crazyMode is True:
            move_speed = 1.0
        Gp.move_move(limb, group, pre_grasp_pos)
        square_list = None

        timeout = 0
        while square_list is None:
            if timeout > 5:
                rospy.logerr("No block exists in the frame. Returning to initial position")
                break
            img = Gp.take_picture(0, 30)
            square_list = iH.square_img_to_centers_list(img)
            number_of_blocks_left = len(square_list)
            timeout += 1

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        if debugMode == 20:
          exit()

        # Move above the desired block to generate better grasp model
        # Offset from gripper to camera
        camera_offset = 0

        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0] + camera_offset, p_y=worldVec[1], p_z=0.32,
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])


    #    Gp.move_move(limb, group, positions=moveJoint)
        Gp.move_move(limb, group, moveJoint)

        # Retake image about the block for recognition
        img = Gp.take_picture(0, 30)
        square_list = iH.square_img_to_centers_list(img)

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        Ang = square_to_find.getAngle(square_list)
        Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1, group, temp_workspace)

        movingLoc = drop_off_location
        pre_moving_loc = copy.deepcopy(drop_off_location)
        pre_moving_loc[2] += 0.15
        Qua = Gp.euler_to_quaternion(z=pre_moving_loc[3])
        pre_drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                                  p_x=pre_moving_loc[0], p_y=pre_moving_loc[1], p_z=pre_moving_loc[2],
                                                  q_x=Qua[0], q_y=Qua[1], q_z=Qua[2], q_w=Qua[3])
        drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                              q_x=Qua[0], q_y=Qua[1], q_z=Qua[2], q_w=Qua[3])
        Gp.move_move(limb, group, pre_drop_block_pos)
        Gp.move_move(limb, group, drop_block_pos, speed_ratio=0.3)

        block = square_list[0]
        block.setIndex(block_index)
        block.setLocation(movingLoc, dQ)
        gripper.open()
        Gp.move_move(limb, group, pre_drop_block_pos)

        # While loop stuff
        moved_times += 1
        block_index += 1
    cv2.destroyAllWindows()
    iterations -= 1

print "Task completed."

if debugMode == 0:
    Gp.move_move(limb, group, safe_move_r2l)
# After finishing task, robot's gripper will move to a secure location
if debugMode == 2:
    move_speed = 0.5
    Gp.move_move(limb, group, safe_move_r2l)  # required ending chuck -- starting
    Gp.remove_objects(scene)
    Gp.remove_camera_w_mount(scene)
    moveComm.roscpp_shutdown()
# required ending chuck -- ending
