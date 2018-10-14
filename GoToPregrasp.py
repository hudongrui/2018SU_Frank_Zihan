import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys

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
# Pre-grasping joint angles
camera_center_human_right = [-0.1047236328125, -0.5071337890625, 1.273416015625, 1.253248046875, -0.9950966796875, 1.472162109375, 2.79098046875]
gripper_center_human_right = [-0.0431328125, -0.4307822265625, 1.257787109375, 1.0017109375, -1.04582421875, 1.554017578125, 2.63585546875]
camera_center_human_left = [-1.228408203125, -0.76178125, 1.41880078125, 1.932666015625, -0.8717197265625, 1.20846875, 2.2628583984375]
gripper_center_human_left = [-1.1013798828125, -0.66011328125, 1.33263671875, 1.671609375, -0.9153974609375, 1.3138095703125, 2.148287109375]

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
# Gp.load_camera_w_mount(scene)

# print limb.joint_angles()
dQ = Gp.euler_to_quaternion(z=3.1415/2)
print dQ
operation_height = 0.25

drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=0.3, p_y=0.780, p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

Gp.move_move(limb, group, drop_block_pos, 0.2)
rospy.sleep(1)

img = Gp.take_picture(0, 30)
square_list = iH.square_img_to_centers_list(img)
number_of_blocks_left = len(square_list)
timeout += 1

#####################################################################################################################
# Here is to take a picture at pre-grasping postion
# frame = Gp.take_picture(0, 30)
# cv2.imshow("Displaying Image", frame)
# cv2.waitKey()
# cv2.destroyAllWindows()
# cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/Background.jpg", frame)
######################################################################################################################
