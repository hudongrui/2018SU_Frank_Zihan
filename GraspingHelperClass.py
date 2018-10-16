import rospy
import math
import cv2
import tf
import numpy as np
import camera_calib_BMW as camCalib
import transformations
import os
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Float64MultiArray,
    Float64
)

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

##############################################
# added by Zihan
from numpy import float64
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint
)

from moveit_msgs.msg import RobotTrajectory

# These imports are used in the train_object main code
import moveit_msgs
import moveit_commander
import moveit_msgs.msg

from math import pi
from moveit_commander.conversions import pose_to_list
from moveit_commander.conversions import pose_to_list
import interceptHelper as iH


def ik_service_client(limb, use_advanced_options, p_x, p_y, p_z, q_x, q_y, q_z, q_w, workspace=True):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    camera_center_human_right = [1.307111328125, -0.383283203125, -0.6499521484375, 0.4143857421875, 0.593916015625, 1.6078818359375, 2.8055830078125]

    camera_center_human_left = [-2.069859375, -0.5306162109375, 1.6159716796875, -0.0359453125, -1.6358544921875, 2.09780078125, -0.366337890625]
    if workspace:
        return_joint = camera_center_human_left
    else:
        return_joint = camera_center_human_right


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


def smooth_move(limb, positions, speed_ratio=None, accel_ratio=None, timeout=None):
    # this move method is obtained from Intera motion control interface
    # limb -- the limb object of the Sawyer robot
    # position -- a list with a length of 7, specifying the goal state joint position
    # speed_ratio -- the speed of robot arm, [0, 1]
    # accel_ratio -- the acceleration of the robot arm, [0, 1]
    # timeout -- the timeout for this specific motion; infinite when None
    rospy.sleep(1)
    if accel_ratio is None:
        accel_ratio = 0.1
    if speed_ratio is None:
        speed_ratio = 0.3
    traj = MotionTrajectory(limb=limb)

    wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,
                                     max_joint_accel=accel_ratio)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

    waypoint.set_joint_angles(joint_angles=positions)
    traj.append_waypoint(waypoint.to_msg())

    traj.send_trajectory(timeout=timeout)
    rospy.sleep(1)


def load_objects(scene, planning_frame):
    # Loading environment objects including left table, frontal desk, wall, and etc.
    rospy.loginfo("Loading environment objects ...")

    rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    box_pose = PoseStamped()
    x = 72
    y = 30
    z = 34
    box_pose.header.frame_id = planning_frame  # must put it in the frame
    box_pose.pose.position.x = in_to_m(-15) + in_to_m(x / 2)
    box_pose.pose.position.y = in_to_m(18) + in_to_m(y / 2)
    box_pose.pose.position.z = in_to_m(-36) + in_to_m(z / 2)
    box_name = "right table"
    scene.add_box(box_name, box_pose, (in_to_m(x), in_to_m(y), in_to_m(z)))

    rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    box_pose = PoseStamped()
    x = 30
    y = 72
    z = 34
    box_pose.header.frame_id = planning_frame  # must put it in the frame
    box_pose.pose.position.x = in_to_m(-23) + in_to_m(x / 2)
    box_pose.pose.position.y = in_to_m(-92) + in_to_m(y / 2)
    box_pose.pose.position.z = in_to_m(-36) + in_to_m(z / 2)
    box_name = "left table"
    scene.add_box(box_name, box_pose, (in_to_m(x), in_to_m(y), in_to_m(z)))

    rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    x = 3
    y = 300
    z = 300
    box_pose = PoseStamped()
    box_pose.header.frame_id = planning_frame  # must put it in the frame
    box_pose.pose.position.x = in_to_m(-23) + in_to_m(x / 2)
    box_pose.pose.position.y = in_to_m(-150) + in_to_m(y / 2)
    box_pose.pose.position.z = in_to_m(-150) + in_to_m(z / 2)
    box_name = "wall"
    scene.add_box(box_name, box_pose, (in_to_m(x), in_to_m(y), in_to_m(z)))

    rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    x = 9 # inches
    y = 20 # inches
    z = 300 # inches
    box_pose = PoseStamped()
    box_pose.header.frame_id = planning_frame  # must put it in the frame
    box_pose.pose.position.x = in_to_m(-23) + in_to_m(x / 2) + in_to_m(2) # the added value is for safety
    box_pose.pose.position.y = in_to_m(48) + in_to_m(-y / 2)
    box_pose.pose.position.z = in_to_m(-150) + in_to_m(z / 2)
    box_name = "pillar"
    scene.add_box(box_name, box_pose, (in_to_m(x), in_to_m(y), in_to_m(z)))

    # rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    # x = 11
    # y = 40
    # z = 41
    # box_pose = PoseStamped()
    # box_pose.header.frame_id = planning_frame  # must put it in the frame
    # box_pose.pose.position.x = in_to_m(-23) + in_to_m(x / 2) + in_to_m(2) # the added value is for safety
    # box_pose.pose.position.y = in_to_m(-20) + in_to_m(y / 2)
    # box_pose.pose.position.z = in_to_m(30) + in_to_m(z / 2)
    # box_name = "visually safe"
    # scene.add_box(box_name, box_pose, (in_to_m(x), in_to_m(y), in_to_m(z)))


    rospy.sleep(1)


def in_to_m(inch):
    return inch * 2.54 / 100


def load_camera_w_mount(scene):
    rospy.loginfo("Loading camera mount")
    rospy.sleep(1)  # this is a must otherwise the node will skip placing the box
    box_pose = PoseStamped()
    link_name = 'right_l6'
    box_pose.header.frame_id = 'right_l6'  # must put it in the frame
    box_pose.pose.position.y = -0.09
    box_name = "camera_w_mount"
    scene.attach_box(link_name, box_name, box_pose, size=(0.08, 0.08, 0.08))
    rospy.sleep(1)


def remove_objects(scene):
    rospy.loginfo("Removing obstacles")
    rospy.sleep(1)
    scene.remove_world_object("right table")
    rospy.sleep(1)
    scene.remove_world_object("wall")
    rospy.sleep(1)
    scene.remove_world_object("pillar")
    rospy.sleep(1)
    scene.remove_world_object("left table")
    rospy.sleep(1)
    # scene.remove_world_object("visually safe")
    # rospy.sleep(1)


def remove_camera_w_mount(scene):
    rospy.loginfo("Removing camera mount")
    rospy.sleep(1)
    grasping_group = 'right_l6'
    scene.remove_attached_object(grasping_group)
    rospy.sleep(1)
    scene.remove_world_object("camera_w_mount")
    rospy.sleep(1)


def move_move(limb, group, target, speed_ratio=None, accel_ratio=None, timeout=None):
    if speed_ratio is None:
        speed_ratio = 0.3
    if accel_ratio is None:
        accel_ratio = 0.1
    plan = group.plan(target)
    rospy.sleep(1)
    step = []
    for point in plan.joint_trajectory.points:
        step.append(point.positions)
    traj = MotionTrajectory(limb=limb)
    wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,
                                     max_joint_accel=accel_ratio)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
    for point in step:
        waypoint.set_joint_angles(joint_angles=point)
        # print point
    traj.append_waypoint(waypoint.to_msg())
    traj.send_trajectory(timeout=timeout)
    group.stop()
    group.clear_pose_targets()


def check_if_attached(scene, box_name, box_is_attached, box_is_known):
    timeout = 5
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


# ======================================================================================
def pixelToWorld(u, v):
    # u is x, v is y
    ## all the ## comment are from Zihan && Frank summer research project
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            # transform from '/gripper' (target) frame to '/base' (source) frame
            (trans, rot) = listener.lookupTransform('/base', '/right_gripper_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if trans is None:
            continue
        else:
            break

        rate.sleep()

    # print trans
    # print rot

    z = trans[2]
    trans = np.array(trans)
    trans.shape = (3, 1)

    # print trans
    
    hom_Mtrx_g_b = transformations.quaternion_matrix(rot) ## using quaternion matrix to get transformation matrix

    ## modifying the x-y-z position vector
    hom_Mtrx_g_b[0][3] = trans[0]
    hom_Mtrx_g_b[1][3] = trans[1]
    hom_Mtrx_g_b[2][3] = trans[2]
    # print hom_Mtrx_g_b

    # homogeneous transformation from /camera to /gripper

    hom_Mtrx_c_g = transformations.rotation_matrix(-math.pi / 2.0, [0, 0, 1], [0, 0, 0]) ## this is the key thing to check and verify
    # hom_Mtrx_c_g = transformations.rotation_matrix(0, (0, 0, 1))
    hom_Mtrx_c_g[0][3] = in_to_m(-2.5) ## this is a guess number from gripper to camera
    # print hom_Mtrx_c_g

    ## this is just calculating the transformation matrix from base to camera
    hom_Mtrx_c_b = np.dot(hom_Mtrx_g_b, hom_Mtrx_c_g)
    # print hom_Mtrx_c_b
    Mtrx_c_b = hom_Mtrx_c_b[:3, :4]
    # print Mtrx_c_b
    Mtrx_c_b = np.matrix(Mtrx_c_b)
    # print Mtrx_c_b

    camMtrx = camCalib.getCamMatrx()
    # print camMtrx
    camMtrxInv = np.linalg.inv(camMtrx)
    # print camMtrxInv
    camMtrxInv = np.matrix(camMtrxInv)
    # print camMtrxInv

    pixVec = np.matrix([[z * u], [z * v], [z * 1]])  # pixel vector augmented by 1
    # print pixVec

    # testVec = camMtrxInv*z*pixVec
    one = np.array([1])
    one.shape = (1, 1)
    camVec = np.concatenate((camMtrxInv * pixVec, one), axis=0)
    # print camVec
    worldVec = Mtrx_c_b * camVec

    return worldVec, hom_Mtrx_c_b, rot


# ======================================================================================


# ======================================================================================
def graspExecute(limb, gripper, W, H, Ang, x_ref, y_ref, table, group, workspace):
    # 0.05 both
    # TODO
    if workspace:
        y_offset = 0.01 - 0.003
        x_offset = 0.045 # 0m is the camera offset from the gripper center
    else:
        y_offset = 0.004 + 0.002
        x_offset = 0.05 - 0.004
    # y_offset = 0.005
    # x_offset = 0.065 + 0 # 0m is the camera offset from the gripper center
    print("Beginning Grasp execute\n----------------------------------------------------------------")
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
    quat2 = transformations.quaternion_about_axis(-np.pi / 2.0, (0, 0, 1))
    # quat1 = transformations.quaternion_about_axis(np.pi, (1, 0, 0))
    #
    quat = transformations.quaternion_multiply(quat3, quat2)  # transformations.quaternion_multiply(quat2, quat1))
    # print(quat)
    # print(transformations.rotation_from_matrix(hom_rotGrasp1))

    angles = limb.joint_angles()
    endEffAng = angles['right_j6'] # currently 128 degree
    # print 'Current Joint 6 Angle in World Frame is: ', endEffAng * 180.0 / np.pi
    targetAng = endEffAng + Ang

    x_target = (endEffPos[0] + x_ref) * 0.5
    y_target = (endEffPos[1] + y_ref) * 0.5
    quat_replace = euler_to_quaternion(z=-Ang)
    top_grasp_joint = ik_service_client(limb='right', use_advanced_options=True,
                                        p_x=x_target + x_offset, p_y=y_target + y_offset, p_z=0.2,
                                        q_x=quat_replace[0], q_y=quat_replace[1], q_z=quat_replace[2],
                                        q_w=quat_replace[3])
    mid_grasp_joint = ik_service_client(limb='right', use_advanced_options=True,
                                        p_x=x_target + x_offset, p_y=y_target + y_offset, p_z=0.15,
                                        q_x=quat_replace[0], q_y=quat_replace[1], q_z=quat_replace[2],
                                        q_w=quat_replace[3])
    down_grasp_joint = ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=x_target + x_offset, p_y=y_target + y_offset, p_z=0.1,
                                         q_x=quat_replace[0], q_y=quat_replace[1], q_z=quat_replace[2],
                                         q_w=quat_replace[3])
    lstTop = list(top_grasp_joint)
    lstMid = list(mid_grasp_joint)
    lstDown = list(down_grasp_joint)
    top_grasp_joint = tuple(lstTop)
    mid_grasp_joint = tuple(lstMid)
    down_grasp_joint = tuple(lstDown)

    # TODO: move_move(limb, group, positions=..., speed_ratio=...)
    move_move(limb, group, top_grasp_joint, speed_ratio=0.3)
    move_move(limb, group, mid_grasp_joint, speed_ratio=0.2)
    move_move(limb, group, down_grasp_joint, speed_ratio=0.2)
    rospy.sleep(1)
    gripper.close()
    move_move(limb, group, top_grasp_joint, speed_ratio=0.2)
    # gripper.open()  // commented out by CRY 10-02-2018
    rospy.sleep(1)
    print("Completing grasp execute\n------------------------------------------------------")
# ======================================================================================


def dropExecute(limb, gripper, drop_off_location, dQ, group, operation_height, temp_workspace):
    Qua = euler_to_quaternion(z=drop_off_location[3])
    x = drop_off_location[0]
    y = drop_off_location[1]
    z = drop_off_location[2]
    top_drop_position = ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=x, p_y=y, p_z=operation_height,
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3],workspace=temp_workspace)
    mid_drop_position = ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=x, p_y=y, p_z=operation_height + z - 0.2,
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3],workspace=temp_workspace)
    down_drop_position = ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=x, p_y=y, p_z=z,
                                          q_x=Qua[0], q_y=Qua[1], q_z=Qua[2], q_w=Qua[3],workspace=temp_workspace)
    move_move(limb, group, top_drop_position)
    move_move(limb, group, mid_drop_position)
    move_move(limb, group, down_drop_position)
    gripper.open()
    move_move(limb, group, mid_drop_position)
    move_move(limb, group, top_drop_position)



# ======================================================================================
def take_picture(camera_port, ramp_frames):
    # "camera_port": Camera port number
    # "ramp_frames": Number of frames to throw away while the camera adjusts to light levels

    # Example Usage:
    # take picture from webcam

    # =================================================
    #
    # =================================================

    # Now we can initialize the camera capture object with the cv2.VideoCapture class.
    # All it needs is the index to a camera port.
    camera = cv2.VideoCapture(camera_port)
    # Captures a single image from the camera and returns it in PIL format
    def get_image():
        # read is the easiest way to get a full image out of a VideoCapture object.
        retval, im = camera.read()
        return im

    # Ramp the camera - these frames will be discarded and are only used to allow v4l2
    # to adjust light levels, if necessary
    for i in xrange(ramp_frames):
        temp = get_image()
    print("Taking image...")
    # Take the actual image we want to keep
    final_im = get_image()
    print("Done")

    camera.release()
    return final_im


# ======================================================================================


# ======================================================================================
def detect_objects(fruit_number, img):
    # take picture from webcam
    # now = datetime.datetime.now()
    # file_name = now.strftime("%Y-%m-%d")
    # img = cv2.imread(str(take_picture(file_name)), 1)
    # img = cv2.imread("/home/Documents/2018-01-27.png", 1)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_range_apple = np.array([40, 70, 100], dtype=np.uint8)  # np.array([10, 150, 70], dtype=np.uint8)
    upper_range_apple = np.array([50, 200, 255], dtype=np.uint8)  # np.array([179, 255, 100], dtype=np.uint8)
    red = [30, 30, 200]

    lower_range_avocado = np.array([10, 10, 5], dtype=np.uint8)
    upper_range_avocado = np.array([70, 50, 110], dtype=np.uint8)
    red2 = [10, 10, 255]

    lower_range_banana = np.array([20, 100, 150], dtype=np.uint8)
    upper_range_banana = np.array([40, 200, 255], dtype=np.uint8)
    yellow = [0, 240, 255]

    lower_range_grape = np.array([20, 50, 0], dtype=np.uint8)
    upper_range_grape = np.array([179, 200, 50], dtype=np.uint8)
    purple = [255, 127, 255]

    lower_range_orange = np.array([0, 160, 125], dtype=np.uint8)
    upper_range_orange = np.array([100, 255, 150], dtype=np.uint8)
    orange1 = [50, 127, 255]

    green = [74, 111, 55]

    lower_range_peach = np.array([0, 40, 130], dtype=np.uint8)
    upper_range_peach = np.array([100, 120, 140], dtype=np.uint8)
    pink = [74, 111, 55]

    def apple():
        center_of_apple = find_element_within_range(img, imgHSV, lower_range_apple, upper_range_apple, red)
        print "You asked for an apple.\n"
        return center_of_apple

    def avocado():
        center_of_avocado = find_element_within_range(img, imgHSV, lower_range_avocado, upper_range_avocado, red2)
        print "You asked for an avocado.\n"
        print center_of_avocado
        return center_of_avocado

    def banana():
        center_of_banana = find_element_within_range(img, imgHSV, lower_range_banana, upper_range_banana, yellow)
        print "You asked for a banana.\n"
        return center_of_banana

    def grape():
        center_of_grape = find_element_within_range(img, imgHSV, lower_range_grape, upper_range_grape, purple)
        print "You asked for a grape.\n"
        return center_of_grape

    def orange():
        center_of_orange = find_element_within_range(img, imgHSV, lower_range_orange, upper_range_orange, orange1)
        print "You asked for an orange.\n"
        return center_of_orange

    def pear():
        center_of_pear = find_element_within_range(img, imgHSV, lower_range_pear, upper_range_pear, green)
        print "You asked for a pear.\n"
        return center_of_pear

    def peach():
        center_of_peach = find_element_within_range(img, imgHSV, lower_range_peach, upper_range_peach, pink)
        print "You asked for a peach.\n"
        return center_of_peach

    # map the inputs to the function blocks
    fruits = {1: apple, 2: avocado, 3: banana, 4: grape, 5: orange, 6: pear, 7: peach,}
    desired_fruit_location = fruits[fruit_number]()

    # cv2.imshow('image', img)
    # cv2.waitKey(0)

    cv2.destroyAllWindows()

    return desired_fruit_location


# ======================================================================================


# ======================================================================================
def find_element_within_range(image, imgHSV, lower_range, upper_range, color):
    mask = cv2.inRange(imgHSV, lower_range, upper_range)

    # cv2.imshow('mask', mask)
    # cv2.waitKey(0)

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
    mask = cv2.erode(mask, element, iterations=2)
    mask = cv2.dilate(mask, element, iterations=2)
    mask = cv2.erode(mask, element)

    useless, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maximumArea = 0
    bestContour = None
    for contour in contours:
        currentArea = cv2.contourArea(contour)
        if currentArea > maximumArea:
            bestContour = contour
            maximumArea = currentArea
    # Create a bounding box around the biggest red object
    x, y, w, h = (0, 0, 0, 0)

    if bestContour is not None:
        x, y, w, h = cv2.boundingRect(bestContour)
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 3)

    if x != 0:
        cv2.circle(image, (x + w / 2, y + h / 2), 3, 3)
        center = (x + w / 2, y + h / 2)
    else:
        center = 0
    # cv2.imshow('mask', image)
    # cv2.waitKey(0)
    return center


# ======================================================================================

# ======================================================================================
# Updated by Frank and Zihan 06/27/2018 15:47

def load_images_from_folder(folder):
    images = []
    file_list = os.listdir(folder)

    def last_5chars(x):
        return x[-5:]

    for filename in sorted(file_list, key=last_5chars):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            images.append(img)
    return images


# def detect_block(image):
#     img_show = image.copy()
#
#     templates = load_images_from_folder("/home/team18/Frank_Ray_Zihan/Templates")
#
#     template_index = 1
#     locations = []
#     numbers = []
#
#     for template in templates:
#
#         w, h, _ = template.shape
#         # All the 6 methods for comparison in a list
#         # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
#         #             'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
#
#         # #Below are the two methods that are working
#         meth = 'cv2.TM_CCOEFF_NORMED'
#         img = img_show.copy()
#         method = eval(meth)
#         # Apply template Matching
#         res = cv2.matchTemplate(img, template, method)
#         min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
#
#         # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
#         if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
#             top_left = min_loc
#         else:
#             top_left = max_loc
#         bottom_right = (top_left[0] + w, top_left[1] + h)
#         # recognized.append([top_left, bottom_right])
#
#         if max_val >= 0.7:
#             if template_index == block_num:
#                 cv2.rectangle(img_show, top_left, bottom_right, (0, 0, 255), 2)
#             else:
#                 cv2.rectangle(img_show, top_left, bottom_right, (0, 255, 0), 2)
#             cv2.putText(img_show, str(template_index), (top_left[0], top_left[1]), cv2.FONT_HERSHEY_DUPLEX, 2,
#                         (0, 255, 255), 3)
#             center_x = top_left[0] + w // 2
#             center_y = top_left[1] + h // 2
#             center = (center_x, center_y)
#             print "Found Block " + str(template_index), "at " + str(center)
#             locations.append(center)
#             numbers.append(template_index)
#
#         template_index = template_index + 1
#
#     if block_num not in numbers:
#         print "Could not find the block you are asking for."
#         print "Please adjust the block so the camera could recognize the block."
#         return None, image
#     else:
#         return locations[numbers.index(block_num)], img_show


# ============================================================================================


# from Henry & Frank
def euler_to_quaternion(x=None, y=None, z=None):
    # Attention all units in radian
    # x - rotation angle about the x axis (heading)
    # y - rotation angle about the y axis (attitude)
    # z - rotation angle about the z axis (bank)
    if x is None:
        x = np.pi
    if y is None:
        y = 0
    if z is None:
        z = np.pi
    else:
        z = z - math.pi
    return transformations.quaternion_from_euler(x, y, z)
