import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys
import time

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

drop_block_pos = camera_center_human_right

start = time.time()
Gp.move_move(limb, group, drop_block_pos, 0.2)
end = time.time()

print end - start

rospy.sleep(1)

# img = Gp.take_picture(0, 30)
# square_list = iH.square_img_to_centers_list(img)
# number_of_blocks_left = len(square_list)
# timeout += 1

#####################################################################################################################
# Here is to take a picture at pre-grasping postion
# frame = Gp.take_picture(0, 30)
# cv2.imshow("Displaying Image", frame)
# cv2.waitKey()
# cv2.destroyAllWindows()
# cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/Background.jpg", frame)
######################################################################################################################
