import subprocess
from multiprocessing import Process
from multiprocessing import Manager
import multiprocessing
import rospy
import graspObjectImageFunc as gi
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import sys
import pose_receiver
import os

def get_ws_direction(data_from_thread):
    print "Point to a direction!"
    subprocess.call('python pose_receiver.py', shell=True)
    # os.system('python pose_receiver.py')
    with open("pose_rec_finish.txt", "rb") as f:
        data_from_thread[0] = int(f.read())
    print "Subprocess Gesture Part DONE."


def listen_to_fruit(data_from_thread):
    # silence please!
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/silence_please_preparing_to_record.JPG")
    rospy.sleep(0.5)
    subprocess.call('python ./NLP_class.py', shell=True)

    # print "Types of fruits: 1, apple; 2, avocado; 3, banana; 5, orange; 7, peach"
    # type_of_fruit = raw_input("Enter the fruit you want: ")
    with open("finalvalue.txt", "rb") as f:
        type_of_fruit = int(f.read())
        data_from_thread[1] = type_of_fruit

    print "Subprocess NLP Part DONE."

if __name__ == '__main__':

    # Below is the original starting code
    rospy.init_node("GraspingDemo")
    global limb
    limb = intera_interface.Limb('right')
    global GRIPPER
    gripper = intera_interface.Gripper('right_gripper')
    gripper.open()
    headDisplay = head.HeadDisplay()
    side_of_user = -1
    type_of_fruit = -1
    manager = Manager()
    data_from_thread = manager.list([-1, -1])

    # Welcome slide
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/hello.JPG")
    rospy.sleep(1)

    # ======================================================================================
    # Reset joint angles
    safe_move_r2l = [-0.504587890625, -1.9217080078125, 0.319630859375, 0.933556640625, 0.12821875, 2.55040625,
                     -1.351919921875]
    # Gesture joint angles
    gest_position = [-0.3003134765625, -1.312572265625, 0.371109375, 0.4870419921875, 0.021302734375, 1.1612021484375,
                     1.62130078125]

    dQ = Gp.euler_to_quaternion(z=0)

    operation_height = 0.443

    ###############################################################
    moveComm = Gp.moveit_commander
    moveComm.roscpp_initialize(sys.argv)

    robot = moveComm.RobotCommander()
    scene = moveComm.PlanningSceneInterface()

    group_name = "right_arm"
    group = moveComm.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   Gp.moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()

    eef_link = group.get_end_effector_link()
    # Gp.load_objects(scene, planning_frame)
    Gp.load_camera_w_mount(scene)
    ###############################################################

    print limb.joint_angles()
    Gp.smooth_move(limb, safe_move_r2l, 0.2)

    # Gesture Recognition and NLP running simultaneously
    p2 = Process(target = get_ws_direction, args = (data_from_thread,))
    p1 = Process(target = listen_to_fruit, args = (data_from_thread,))

    p1.start()
    p2.start()
    p1.join()
    p2.join()
    # listen_to_fruit()
    side_of_user = data_from_thread[0]
    type_of_fruit = data_from_thread[1]

    # ges = PoseRec.PoseRecognition()
    if type_of_fruit == 1:
        # I heard apple
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_apple.JPG")
    elif type_of_fruit == 2:
        # I heard avocado
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_avocado.JPG")
    elif type_of_fruit == 3:
        # I heard banana
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_banana.JPG")
    elif type_of_fruit == 5:
        # I heard orange
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_orange.JPG")
    else:
        # I heard peach
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_said_peach.JPG")
    rospy.sleep(2.5)

    dQ = Gp.euler_to_quaternion(z=0)


    # x = 36 inch, y = -18 inch
    camera_center_human_right = [Gp.in_to_m(20), Gp.in_to_m(-17), operation_height - 0.343, 0]
    # Gp.ik_service_client(limb='right', use_advanced_options=True,
    #                                           p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(-17), p_z=operation_height - 0.343,
    #                                           q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

    # x = 36 inch, y = 18 inch
    camera_center_human_left = [Gp.in_to_m(20), Gp.in_to_m(16), operation_height - 0.343, 0]
    # Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              # p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(16), p_z=operation_height - 0.343,
                                              # q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
    if side_of_user == 1:
        drop_fruit_pos = camera_center_human_right

    else:
        drop_fruit_pos = camera_center_human_left


    table = str(side_of_user)
    if side_of_user == 1:
        # left side of user
        pre_grasp_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(17), p_z=operation_height,
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        # I saw you point left
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_pointed_left.JPG")
    elif side_of_user == 0:
        # right side of user
        pre_grasp_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                              p_x=Gp.in_to_m(20), p_y=Gp.in_to_m(-17), p_z=operation_height,
                                              q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
        # I saw you point right
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/you_pointed_right.JPG")
        # Gp.move(limb, safe_move_r2l, 0.2)
    else:
        print "Side of User is not populated."
        sys.exit()



    Gp.smooth_move(limb, pre_grasp_pos, 0.2)


    frame = Gp.take_picture(0, 30)  # changed to port 1 cry 15/02/2018

    objLoc = Gp.detect_objects(type_of_fruit, frame)

    # print objLoc

    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

    # print worldVec
    # TODO: change values on offset

    if side_of_user == 1:
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])
        workspace = True
    else:
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1] - 0.07, p_z=0.5,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])
        workspace = False

    # moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
    #                                  p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,  # q_x=0, q_y=0, q_z=0, q_w=0)
    #                                  q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

    Gp.smooth_move(limb, moveJoint, 0.2)

    frame = Gp.take_picture(0, 30)

    objLoc = Gp.detect_objects(type_of_fruit, frame)

    H, W, Ang = gi.predictGraspOnImage(frame, objLoc)




    Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], table, group, workspace)
    # rospy.sleep(1)
    # rospy.sleep(1)

    Gp.dropExecute(limb, gripper, drop_fruit_pos, dQ, group, operation_height, workspace)
    # rospy.sleep(0.5)
    # gripper.open()
    rospy.sleep(0.5)
    Gp.smooth_move(limb, safe_move_r2l, 0.2)
    rospy.sleep(0.5)

    # Done
    headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/all_done.JPG")
    rospy.sleep(1)
    image_gest = cv2.imread("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg", 1)
    resize_img = cv2.resize(image_gest, (1024, 600))
    cv2.imwrite("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg", resize_img)
    headDisplay.display_image("/home/team18/openpose/examples/Sawyer_GesRec/input_dir/webcam.jpeg")
