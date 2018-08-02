import rospy
import intera_interface

rospy.init_node('GetPose')

limb = intera_interface.Limb('right')


robotPose = limb.endpoint_pose()
print robotPose

print robotPose.get('position').x

print limb.joint_ordered_angles()

