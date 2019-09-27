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
import gc

debugMode = 1800

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
dQ = Gp.euler_to_quaternion(z=0)

operation_height = 0.443

# # x = 5 inch = 0.13m y = 33 inch = 0.83m
# camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                           p_x=0.13, p_y=0.55, p_z=operation_height,
#                                           q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
# # camera_center_human_right = [0.7406572265625, -0.67773046875, 1.38037890625, 0.7668037109375, -0.9584423828125, 1.9225859375, -2.98408203125]

# # x = -5 inch = -0.13m y = -36 inch = -0.91
# camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
#                                           p_x=-0.13, p_y=-0.55, p_z=operation_height,
#                                           q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
# # camera_center_human_left = [-2.122291015625, -0.5280498046875, 1.63117578125, 0.0682919921875, -1.590201171875, 2.102138671875, -0.297224609375]

##################################################################################
#
# TODO: IMPORTANT!!!!! Below are the positions for the temporary setup to address robot unable to avoid collision
#
##################################################################################

# x = 36 inch, y = -18 inch
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(-17), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

# x = 36 inch, y = 18 inch
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(16), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

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
pre_grasp_pos = camera_center_human_left
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
# Gp.load_objects(scene, planning_frame)
Gp.load_camera_w_mount(scene)
###############################################################
if debugMode == 18:
  exit()
square_list = 0

number_of_blocks_left = 0
block_index = 0

# false indicates pickup location is on the left and true is right
temp_workspace = True
iterations = 12  # how many times does the robot have to repeatedly grab the blocks
task_num = 0
if debugMode == 0:
    Gp.move_move(limb, group, safe_move_r2l)

# Read tasks from folder
tasks = iH.read_tasks_from_file()

while iterations != 0:
    # Below returns a set of coordinates where we want to drop the block.
    # TODO choose below
    task = iH.get_loc_by_pixel(temp_workspace, tasks[task_num])
    # task = iH.drop_destinations(temp_workspace)

    print "Executing iteration " + str(task_num + 1)

    if temp_workspace:
        pre_grasp_pos = camera_center_human_right
    else:
        pre_grasp_pos = camera_center_human_left
    # Switch workspace for the next time

    for drop_off_location in task:
        # Pre-grasping joint angles
        move_speed = 0.5
        if crazyMode is True:
            move_speed = 1.0
        Gp.move_move(limb, group, pre_grasp_pos)
        square_list = None

        timeout = 0
        while square_list is None:
            if timeout > 20:
                rospy.logerr("No block exists in the frame. Returning to initial position")
                exit()
            img = Gp.take_picture(0, 30)
            square_list = iH.square_img_to_centers_list(img, temp_workspace)
            timeout += 1

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        if debugMode == 20:
          exit()

        # Move above the desired block to generate better grasp model
        # Offset from gripper to camera
        camera_offset = 0

        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0] + camera_offset, p_y=worldVec[1], p_z=operation_height,
                                         q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3],workspace=temp_workspace)


    #    Gp.move_move(limb, group, positions=moveJoint)
        Gp.move_move(limb, group, moveJoint)
        gc.collect()
        # Retake image about the block for recognition
        img = Gp.take_picture(0, 30)
        square_list = iH.square_img_to_centers_list(img, temp_workspace)

        timeout = 0
        while square_list is None:
            if timeout > 20:
                rospy.logerr("No block exists in the frame. Returning to initial position")
                exit()
            img = Gp.take_picture(0, 30)
            square_list = iH.square_img_to_centers_list(img, temp_workspace)
            timeout += 1

        square_to_find = iH.find_square_closest_to_center(img, square_list)

        W = square_to_find.getCenter().x
        H = square_to_find.getCenter().y
        # H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
        worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
        Ang = square_to_find.getAngle(square_list)
        Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1, group, temp_workspace)
        # Gp.move_move(limb, group, moveJoint)

        Gp.dropExecute(limb,gripper,drop_off_location, dQ, group, operation_height, temp_workspace)

        block = square_list[0]
        block.setIndex(block_index)
        block.setLocation(drop_off_location, dQ)
        # While loop stuff
        block_index += 1
        # free(img)
        gc.collect()
    temp_workspace = not temp_workspace
    # cv2.destroyAllWindows()
    iterations -= 1
    task_num += 1
    gc.collect()

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
