import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys

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
operation_height = 0.443
# x = 36 inch, y = -18 inch
camera_center_human_right = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(-19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

gripper_center_human_right = [-0.0431328125, -0.4307822265625, 1.257787109375, 1.0017109375, -1.04582421875, 1.554017578125, 2.63585546875]
# x = 36 inch, y = 18 inch
camera_center_human_left = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=Gp.in_to_m(25), p_y=Gp.in_to_m(19), p_z=operation_height,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
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
Gp.load_objects(scene, planning_frame)
Gp.load_camera_w_mount(scene)

# print limb.joint_angles()
dQ = Gp.euler_to_quaternion()

Gp.move_move(limb, group, camera_center_human_left, 0.2)