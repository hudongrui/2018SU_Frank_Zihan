import rospy
import roslib
import intera_interface
import math
import time
import tf
import numpy as np
import cv2
import camera_calib_BMW as camCalib
import transformations
import grasp_image_func as gi

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


# ======================================================================================
def ik_service_client(limb, use_advanced_options, p_x, p_y, p_z, q_x, q_y, q_z, q_w):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    # return_joint sends robot to pre-grasp arm configuration
    return_joint = [-1.500208984375, -1.0860322265625, -0.177197265625, 1.3819580078125, 0.0950634765625,
                    1.3055205078125, 1.6654560546875]

    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=p_x,  # pre-grasp:  0.5609462822565073
                    y=p_y,  # -0.3446309287328617
                    z=p_z,
                    # 0.45777571205785983[-1.500208984375, -1.0860322265625, -0.177197265625,
                    # 1.3819580078125, 0.0950634765625, 1.3055205078125, 1.6654560546875]
                ),
                orientation=Quaternion(
                    x=q_x,  # 0.20778492941475438
                    y=q_y,  # 0.9778261053583365
                    z=q_z,  # -0.010705090881921715
                    w=q_w,  # -0.023810330049445317
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return return_joint

    # Check if result valid, and type of seed ultimately used to get solution
    if resp.result_type[0] > 0:
        seed_str = {
            ikreq.SEED_USER: 'User Provided Seed',
            ikreq.SEED_CURRENT: 'Current Joint Angles',
            ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
        }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                      (seed_str,))
        # print("------------------")

        # print('J0: ', resp.joints[0].position[0])
        # print('J1: ', resp.joints[0].position[1])
        # print('J2: ', resp.joints[0].position[2])
        # print('J3: ', resp.joints[0].position[3])
        # print('J4: ', resp.joints[0].position[4])
        # print('J5: ', resp.joints[0].position[5])
        # print('J6: ', resp.joints[0].position[6])

        # move(resp.joints[0].position)
        return_joint = resp.joints[0].position
        return return_joint
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        return return_joint
# ======================================================================================


# ======================================================================================
def move(limb, positions, move_speed):
    limb.set_joint_position_speed(speed=move_speed)

    angles = limb.joint_angles()

    angles['right_j0'] = positions[0]
    angles['right_j1'] = positions[1]
    angles['right_j2'] = positions[2]
    angles['right_j3'] = positions[3]
    angles['right_j4'] = positions[4]
    angles['right_j5'] = positions[5]
    angles['right_j6'] = positions[6]

    limb.move_to_joint_positions(angles)
# ======================================================================================


# ======================================================================================
def pixelToWorld(u, v):
    # u is x, v is y
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # transform from '/gripper' (target) frame to '/base' (source) frame
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

    z = trans[2]
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

    # TODO: change this value to address camera offset in x direction
    hom_Mtrx_c_g[0][3] = 0.05
    # TODO: change this value to address camera offset in y direction
    hom_Mtrx_c_g[1][3] = 0.01
    # TODO: change this value to address camera offset in z direction
    hom_Mtrx_c_g[2][3] = 0.07
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

    return worldVec, hom_Mtrx_c_b, rot
# ======================================================================================


# ======================================================================================
def graspExecute(limb, W, H, Ang):
    [endEffPos, hom_Mtrx_c_b, rotOriginal] = pixelToWorld(W, H)
    # print('endEffPos, x: ', endEffPos[0])
    # print('endEffPos, y: ', endEffPos[1])
    # print('endEffPos, z: ', endEffPos[2])
    hom_rotGrasp = transformations.rotation_matrix(Ang, (0, 0, 1))
    hom_rotGrasp1 = np.dot(hom_Mtrx_c_b, hom_rotGrasp)
    # hom_rotGrasp1[0][3] = 0
    # hom_rotGrasp1[1][3] = 0
    # hom_rotGrasp1[2][3] = 0
    # print (hom_rotGrasp1)
    # quat = transformations.quaternion_from_matrix(hom_Mtrx_c_b)
    quat3 = transformations.quaternion_about_axis(Ang, (0, 0, 1))
    quat2 = transformations.quaternion_about_axis(-np.pi/2.0, (0, 0, 1))
    # quat1 = transformations.quaternion_about_axis(np.pi, (1, 0, 0))
    #
    quat = transformations.quaternion_multiply(quat3, quat2)# transformations.quaternion_multiply(quat2, quat1))
    # print(quat)
    # print(transformations.rotation_from_matrix(hom_rotGrasp1))

    angles = limb.joint_angles()
    endEffAng = angles['right_j6']
    print 'Current Joint 6 Angle in World Frame is: ', endEffAng * 180.0 / np.pi
    targetAng = endEffAng-(np.pi/2.0)+Ang+(np.pi/2.0)

    top_grasp_joint = ik_service_client(limb='right', use_advanced_options=True,
                                        p_x=endEffPos[0], p_y=endEffPos[1], p_z=0.5,  # q_x=0, q_y=0, q_z=0, q_w=0)
                                        q_x=rotOriginal[0], q_y=rotOriginal[1], q_z=rotOriginal[2], q_w=rotOriginal[3])

    down_grasp_joint = ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=endEffPos[0], p_y=endEffPos[1], p_z=0.1,
                                         q_x=rotOriginal[0], q_y=rotOriginal[1], q_z=rotOriginal[2], q_w=rotOriginal[3])

    lstTop = list(top_grasp_joint)
    lstDown = list(down_grasp_joint)
    print 'Grasp Angle in World Frame is: ', targetAng * 180.0 / np.pi
    lstTop[6] = targetAng
    lstDown[6] = targetAng
    top_grasp_joint = tuple(lstTop)
    down_grasp_joint = tuple(lstDown)

    move(limb, positions=top_grasp_joint, move_speed=0.2)
    rospy.sleep(2)
    move(limb, positions=down_grasp_joint, move_speed=0.2)
    rospy.sleep(2)
    gripper.close()
    rospy.sleep(1)
    move(limb, positions=top_grasp_joint, move_speed=0.2)
    gripper.open()
# ======================================================================================


# ======================================================================================
rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right')
gripper.open()

cap = cv2.VideoCapture(1)
# loopCounter = 1
# graspPerm = False
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if frame is None:
        continue
    else:
        H, W, Ang = gi.predictGraspOnImage(frame)
        print(H)  # y-coord of pixel position
        print(W)  # x-coord of pixel position
        print 'Grasp Angle in Cam Frame is: ', Ang*180.0/np.pi  # angle of grasp

    usrInput = raw_input('Does the grasp look good? [enter] = yes, [n] = no')
    if usrInput == 'n':
        print('returning to loop...')
    else:
        print('executing grasp')
        print('----------->')
        #graspExecute(limb, W, H, Ang)
        break

    # usrInput = raw_input('Press enter to continue or "q" to quit')
    # if usrInput == 'q':
    #     print('exiting...')
    #     break
    # else:
    #     print('return to loop')
    #     print('----------->')
    #     # loopCounter = loopCounter + 1

cap.release()
cv2.destroyAllWindows()
