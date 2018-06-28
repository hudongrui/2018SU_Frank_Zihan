import rospy
import numpy as np
import camera_calib_BMW as camCalib
import intera_interface
import math
import tf
import transformations
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
rospy.init_node('Sawyer_wrist_cam_tf_listener')
listener = tf.TransformListener()
rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('/base', '/right_gripper_base', rospy.Time(0))
        print('trans is', trans)
        print('rot is', rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    if trans is None:
        continue
    else:
        break

    rate.sleep()

trans = np.array(trans)
trans.shape = (3, 1)
print(trans)
hom_Mtrx_g_b = transformations.quaternion_matrix(rot)
hom_Mtrx_g_b[0][3] = trans[0]
hom_Mtrx_g_b[1][3] = trans[1]
hom_Mtrx_g_b[2][3] = trans[2]
print("homogeneous transformation from /gripper_base to /base is:")
print(hom_Mtrx_g_b)
# rot_matrix = hom_rot_matrix[:3, :3]
# print(rot_matrix)

hom_Mtrx_c_g = transformations.rotation_matrix(-math.pi/2.0, [0, 0, 1], [0, 0, 0])
hom_Mtrx_c_g[0][3] = 0.05
print("homogeneous transformation from /camera to /gripper_base is:")
print(hom_Mtrx_c_g)

hom_Mtrx_c_b = np.dot(hom_Mtrx_g_b, hom_Mtrx_c_g)
print("homogeneous transformation from /camera to /base is:")
print(hom_Mtrx_c_b)

Mtrx_c_b = hom_Mtrx_c_b[:3, :4]
Mtrx_c_b = np.matrix(Mtrx_c_b)
print("transformation from /camera to /gripper_base is:")
print(Mtrx_c_b)

camMtrx = camCalib.getCamMatrx()
camMtrxInv = np.linalg.inv(camMtrx)
camMtrxInv = np.matrix(camMtrxInv)

z = 0.5  # height of camera above the table
pixVec = np.matrix([[z*278], [z*215], [z*1]])  # pixel vector augmented by 1

# testVec = camMtrxInv*z*pixVec
one = np.array([1])
one.shape = (1, 1)
camVec = np.concatenate((camMtrxInv*pixVec, one), axis=0)
print(camVec)
print(Mtrx_c_b*camVec)


# print(np.concatenate((np.dot(camMtrxInv, np.dot(z, [pixVec])), np.array([1])), axis=0))

# Mext = np.concatenate((rot_matrix, trans), axis=1)
# print(Mext)

# def tf_listener():
#     listener = tf.TransformListener()
#
# 	rate = rospy.Rate(10.0)
# 	while not rospy.is_shutdown():
# 	    try:
# 		    (trans,rot) = listener.lookupTransform('/base', '/ar_marker_5', rospy.Time(0))
# 	    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
# 		    continue
#
# 	    return trans
#
# 	    rate.sleep()