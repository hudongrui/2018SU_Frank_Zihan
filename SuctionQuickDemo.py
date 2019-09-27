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
# print dQ
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
for i in range (3):
    print "Closing Gripper"
    Gp.suctionClose()
    rospy.sleep(3)
    print "Openning Gripper"
    Gp.suctionOpen()
    rospy.sleep(3)

print "Task completed."
