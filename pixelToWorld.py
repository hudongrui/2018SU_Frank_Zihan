import rospy
import math
import tf
import transformations
import numpy as np
import camera_calib_BMW as camCalib


def pixelToWorld(u, v, z):
    # u, v are x, y coord in pixel frame respectively, z is the height of the camera above the table

    rospy.init_node('Sawyer_wrist_cam_tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base', '/right_gripper_base', rospy.Time(0))
            # print('trans is', trans)
            # print('rot is', rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if trans is None:
            continue
        else:
            break

        rate.sleep()

    trans = np.array(trans)
    trans.shape = (3, 1)
    # print(trans)
    hom_Mtrx_g_b = transformations.quaternion_matrix(rot)
    hom_Mtrx_g_b[0][3] = trans[0]
    hom_Mtrx_g_b[1][3] = trans[1]
    hom_Mtrx_g_b[2][3] = trans[2]
    # print("homogeneous transformation from /gripper_base to /base is:")
    # print(hom_Mtrx_g_b)

    hom_Mtrx_c_g = transformations.rotation_matrix(-math.pi / 2.0, [0, 0, 1], [0, 0, 0])
    hom_Mtrx_c_g[0][3] = 0.05
    # print("homogeneous transformation from /camera to /gripper_base is:")
    # print(hom_Mtrx_c_g)

    hom_Mtrx_c_b = np.dot(hom_Mtrx_g_b, hom_Mtrx_c_g)
    # print("homogeneous transformation from /camera to /base is:")
    # print(hom_Mtrx_c_b)

    Mtrx_c_b = hom_Mtrx_c_b[:3, :4]
    Mtrx_c_b = np.matrix(Mtrx_c_b)
    # print("transformation from /camera to /gripper_base is:")
    # print(Mtrx_c_b)

    camMtrx = camCalib.getCamMatrx()
    camMtrxInv = np.linalg.inv(camMtrx)
    camMtrxInv = np.matrix(camMtrxInv)

    # z = 0.5  # height of camera above the table
    pixVec = np.matrix([[z * u], [z * v], [z * 1]])  # pixel vector augmented by 1

    # testVec = camMtrxInv*z*pixVec
    one = np.array([1])
    one.shape = (1, 1)
    camVec = np.concatenate((camMtrxInv * pixVec, one), axis=0)
    worldVec = Mtrx_c_b * camVec
    # print(camVec)
    # print(Mtrx_c_b * camVec)

    return worldVec
