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
dQ = Gp.euler_to_quaternion(z=2*3.14159)
# print dQ
operation_height = 0.443

# x = 5 inch = 0.13m y = 33 inch = 0.83m
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=0.13, p_y=0.83, p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

# x = -5 inch = -0.13m y = -36 inch = -0.91
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=-0.13, p_y=-0.91, p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])


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
gripper.close()