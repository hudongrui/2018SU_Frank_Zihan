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

################################################
## Start up code to initialize robot

rospy.init_node("FrankZihanSu")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

dQ = Gp.euler_to_quaternion(z=0)
# print dQ
operation_height = 0.443

# x = 36 inch, y = -18 inch
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(-19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

# x = 36 inch, y = 18 inch
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]
test_location = [1.56305078125, 0.09955078125, 2.441650390625, -1.2888828125, -1.8209716796875,
                 0.7058720703125, -0.894408203125]
collision_move = [-1.949134765625, 0.5128837890625, -3.0382353515625, 1.2715830078125, 2.7810556640625,
                  1.3979755859375, 1.5122421875]
home_position = [0,0,0,0,0,0,0 + 10/180*3.1415]                                       

pre_grasp_pos = camera_center_human_left

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

####################################################

number_of_blocks_left = 0
block_index = 0

# false indicates pickup location is on the left and true is right
temp_workspace = True
iterations = 1  # how many times does the robot have to repeatedly grab the blocks

while iterations != 0:

    if temp_workspace:
        pre_grasp_pos = camera_center_human_right
    else:
        pre_grasp_pos = camera_center_human_left

    # Grab the block, move to another table, perform recognition to identify marked area,
    # Drop off the block at that location
    Gp.move_move(limb, group, pre_grasp_pos)


    # Looking for the block
    square_list = None
    #Use Gp.take_picture(0, 30) for webcam and Gp.get_video() for Kinect. Make sure to replace all 4 instances
    timeout = 0
    while square_list is None:
        if timeout > 10:
            rospy.logerr("No block exists in the frame. Returning to initial position")
            exit()
        img = Gp.get_video()
        square_list = iH.square_img_to_centers_list(img, temp_workspace)
        number_of_blocks_left = len(square_list)
        timeout += 1

    square_to_find = iH.find_square_closest_to_center(img, square_list)

    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())

     # Move above the desired block to generate better grasp model
    # Offset from gripper to camera
    camera_offset = 0

    moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                     p_x=worldVec[0] + camera_offset, p_y=worldVec[1], p_z=operation_height,
                                     q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3],workspace=temp_workspace)

    # Prepare to pick up the block
    Gp.move_move(limb, group, moveJoint)

    # Retake image about the block for recognition
    img = Gp.get_video()
    square_list = iH.square_img_to_centers_list(img, temp_workspace)

    timeout = 0
    while square_list is None:
        if timeout > 20:
            rospy.logerr("No block exists in the frame. Returning to initial position")
            exit()
        img = Gp.get_video()
        square_list = iH.square_img_to_centers_list(img, temp_workspace)
        timeout += 1

    square_to_find = iH.find_square_closest_to_center(img, square_list)

    H, W, Ang = gi.predictGraspOnImage(img, [square_to_find.getCenter().x, square_to_find.getCenter().y])
    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(square_to_find.getCenterX(), square_to_find.getCenterY())
    Ang = square_to_find.getAngle(square_list)
    Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1, group, temp_workspace)

    # Get the marked position and drop the block
    if temp_workspace:
        pre_drop_pos = camera_center_human_left
    else:
        pre_drop_pos = camera_center_human_right

    Gp.move_move(limb, group, pre_drop_pos)
    rospy.sleep(1)

    img = Gp.get_video()
    drop_off_location = iH.get_marked_location(img, temp_workspace)


    H, W, Ang = gi.predictGraspOnImage(img, [drop_off_location.getCenter().x, drop_off_location.getCenter().y])
    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(drop_off_location.getCenterX(), drop_off_location.getCenterY())
    Ang = 0

    Gp.dropBlockByImageExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], 1, group, temp_workspace)

    # While loop recur
    # temp_workspace = not temp_workspace
    # cv2.destroyAllWindows()
    iterations -= 1
    # cv2.waitKey()
    print "Please measure the offsets, press any key when done"


print "Task completed"