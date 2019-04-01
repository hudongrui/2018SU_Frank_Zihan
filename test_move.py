import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys
import interceptHelper as iH
import transformations as tfs
import numpy as np

############################################################
# This script takes the robot to pre-grasp position,
# takes a picture right on top of the workspace, and save it.
# This script is primarily used for debugging.
############################################################

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()
rospy.sleep(1)

# print limb.joint_angles()
dQ = Gp.euler_to_quaternion(z=0)
# print dQ
operation_height = 0.443

##################################################################################
#
# TODO: IMPORTANT!!!!! Below are the positions for the temporary setup to address robot unable to avoid collision
# Previous location is commented in Grasp2D_Demo.py
#
##################################################################################

# x = 36 inch, y = -18 inch
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(-19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
camera_center_human_right_joint_angles = [0.72934375, -0.0131015625, -2.2375146484375, 0.507642578125, 2.1652548828125, 1.8803798828125, -1.083087890625]

# x = 36 inch, y = 18 inch
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

camera_center_human_left_joint_angles = [0.7083759765625, -0.0957626953125, -1.681681640625, 0.980623046875, 1.55379296875, 1.706056640625, 1.4645380859375]

test_desire_position_right = [-1.1520244140625, 0.659958984375, -0.6534267578125, 1.24351171875, 0.2280849609375, -0.2111396484375, 1.0104970703125]

test_desire_position_left = [0.8723349609375, -0.149564453125, 2.60210546875, -1.9059404296875, 2.7951025390625, -0.1133779296875, 2.9656787109375]


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

pre_grasp_pos = camera_center_human_right
Gp.move_move(limb, group, pre_grasp_pos)
# Gp.avoid_move(group, camera_center_human_right)
rospy.sleep(1)