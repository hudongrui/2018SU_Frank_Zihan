import rospy
import intera_interface
import GraspingHelperClass as Gp

rospy.init_node('safeMove')
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()

safeMove = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                 -1.351919921875]

Gp.smooth_move(limb, safeMove, 0.2)