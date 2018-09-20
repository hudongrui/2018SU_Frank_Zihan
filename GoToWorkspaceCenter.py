import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
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
Gp.load_objects(scene, planning_frame)
Gp.load_camera_w_mount(scene)

# print limb.joint_angles()
dQ = Gp.euler_to_quaternion()

Gp.move_move(limb, group, camera_center_human_left, 0.2)