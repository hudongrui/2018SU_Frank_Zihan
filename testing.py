import rospy
import intera_interface
import intera_joint_trajectory_action
import image_geometry


camera = image_geometry.PinholeCameraModel

rospy.init_node('SawyerFwdKinTester')

server = intera_joint_trajectory_action.JointTrajectoryActionServer()
limb = intera_interface.Limb('right')
limb.set_joint_position_speed(speed=.3)

angles = limb.joint_angles()

angles['right_j0'] = 0.166408203125
angles['right_j1'] = 1.1797958984375
angles['right_j2'] = -0.025630859375
angles['right_j3'] = -2.6069521484375
angles['right_j4'] = 0.0394296875
angles['right_j5'] = -0.264345703125
angles['right_j6'] = -0.0020302734375

limb.move_to_joint_positions(angles)



