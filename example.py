import rospy
import intera_interface
rospy.init_node('Hello_Sawyer')
limb = intera_interface.Limb('right')
angles = limb.joint_angles()
limb.move_to_neutral()

# angles['right_j0']=0.0
# angles['right_j1']=0.0
# angles['right_j2']=0.0
# angles['right_j3']=0.0
# angles['right_j4']=0.0
# angles['right_j5']=0.0
# angles['right_j6']=0.0


# limb._command_msg.mode = 1
# limb.move_to_joint_positions(angles)


name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
position1 = [0.15, -1.1788818359375, -0.0011337890625, 1, -0.0011611328125, 0.5675048828125, 3.312234375]
position2 = [0.3, -1.1788818359375, -0.0011337890625, 0, -0.0011611328125, 0.5675048828125, 3.312234375]
velocities = [0, 0, 0, 0, 0, 0, 0]
accelerations = [0, 0, 0, 0, 0, 0, 0]

limb.set_joint_trajectory(name, position1, velocities, accelerations)
rospy.sleep(1)
limb.set_joint_trajectory(name, position2, velocities, accelerations)
