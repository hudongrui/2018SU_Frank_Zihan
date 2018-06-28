import intera_interface
import rospy
import numpy as np

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')

angles = limb.joint_angles()
print angles
endEffAng = angles['right_j6']
print endEffAng*180/np.pi
targetAng = endEffAng - (np.pi / 2.0) + 1

angles['right_j6'] = targetAng
print angles
