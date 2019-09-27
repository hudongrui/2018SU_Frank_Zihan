import rospy
import intera_interface
import GraspingHelperClass as Gp

rospy.init_node('safeMove')
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()

# safeMove = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
#                 -1.351919921875]


safeMove = [-0.3003134765625, -1.312572265625, 0.371109375, 0.4870419921875, 0.021302734375, 1.1612021484375,
                 1.62130078125]




Gp.smooth_move(limb, safeMove, 0.2)