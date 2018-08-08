import rospy
import graspObjectImageFunc as gi
import intera_interface
import GraspingHelperClass as Gp


# import GestRecB
# ======================================================================================
rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right')
gripper.open()

print "Which side of the table: 1, left; 0, right"
table_side = raw_input("Enter the side of the table you want: ")

if table_side == "1":
    # User left-side
    pre_grasp_pos = [-1.7220224609375, -1.1679736328125, -0.138830078125, 1.015201171875, 0.00028515625,
                     1.7164111328125,
                     -1.8061787109375]
else:
    # User right-side
    pre_grasp_pos = [0.8807666015625, -1.1366826171875, 0.9580830078125, 0.99812890625, -0.3362646484375,
                     1.837322265625,
                     -1.4236279296875]

Gp.smooth_move(limb, pre_grasp_pos, 0.3)
# print limb.joint_angles()
while 1:
    print "Types of fruits: 1, apple; 2, avocado; 3, banana; 4, grapes; 5, orange; 6, pear; 7, peach"
    type_of_fruit = raw_input("Enter the fruit you want: ")

    #
    # GESTURE RECOGNITION
    #

    # inputDir = "examples/Sawyer_GesRec/input_dir"
    # outputDir = "examples/Sawyer_GesRec/output_dir"
    # ges = GestRecB.GestureRecognition()
    # side_of_user = ges.execute(inputDir, outputDir)
    # print "1 = left table, 2 = right table"
    # side_of_user = 1  # raw_input("From which table?  ")
    # if side_of_user == 1:
    #     pre_grasp_pos = [-1.7220224609375, -1.1679736328125, -0.138830078125, 1.015201171875, 0.00028515625,
    #                      1.7164111328125,
    #                      -1.8061787109375]
    # else:
    #     pre_grasp_pos = [0.8807666015625, -1.1366826171875, 0.9580830078125, 0.99812890625, -0.3362646484375,
    #                      1.837322265625,
    #                      -1.4236279296875]
    # print limb.joint_angles()
    # Gp.move(limb, pre_grasp_pos, 0.2)

    #
    # GESTURE RECOGNITION
    #

    frame = Gp.take_picture(1, 30)

    objLoc = Gp.detect_objects(int(type_of_fruit), frame)

    # print objLoc

    worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

    # print worldVec

    if table_side == "1":
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])
    else:
        moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                         p_x=worldVec[0], p_y=worldVec[1] - 0.08, p_z=0.5,
                                         # q_x=0, q_y=0, q_z=0, q_w=0)
                                         q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

    Gp.smooth_move(limb, positions=moveJoint, move_speed=0.2)

    frame = Gp.take_picture(1, 30)

    # gscale = 0.234375
    # imsize = max(frame.shape[:2])
    # gsize = int(gscale * imsize)  # Size of grasp patch
    #
    # print "image shape is:", frame.shape

    # cv2.imshow('image', frame)
    # cv2.waitKey(0)

    objLoc = Gp.detect_objects(int(type_of_fruit), frame)

    # try:
    #     H, W, Ang = gi.predictGraspOnImage(frame, objLoc)
    # except ValueError:
    #     print('failed 1st grasping attempt')

    # frame = Gp.take_picture(0, 30)
    # cv2.imshow('image', frame)
    # cv2.waitKey(0)

    H, W, Ang = gi.predictGraspOnImage(frame, objLoc)
    # H = (H+worldVec[0])*0.5
    # W = (W+worldVec[1])*0.5

    # print "W is: ", W
    # print "H is: ", H
    # print "worldVec[0] is: ", worldVec[0]
    # print "worldVec[1] is: ", worldVec[1]

    Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1], table_side)

    rospy.sleep(1)
    gripper.open()
    Gp.smooth_move(limb, pre_grasp_pos, 0.2)



# # cap = cv2.VideoCapture(0)
# # rospy.sleep(2)
# # # loopCounter = 1
# # # graspPerm = FalseZ
# while True:
#     # Capture frame-by-frame
#     frame = Gp.take_picture(0, 30)
#     if frame is None:
#         continue
#     else:
#         H, W, Ang = gi.predictGraspOnImage(frame, objLoc)
#         print 'from object detection, desired object is at x(W) = ', objLoc[0]
#         print 'from object detection, desired object is at y(H) = ', objLoc[1]
#         print 'from graspNet prediction, grasp is at x = ', W  # x-coord of pixel position
#         print 'from graspNet prediction, grasp is at y = ', H  # y-coord of pixel position
#         print 'Grasp Angle in Cam Frame is: ', Ang * 180.0 / np.pi  # angle of grasp
#
#     usrInput = raw_input('Does the grasp look good? [enter] = yes, [n] = no')
#     if usrInput == 'n':
#         print('returning to loop...')
#     else:
#         print('executing grasp')
#         print('----------->')
#         # aggregate the object location in pixel coordinates from object detection and grasp generation
#         # W = (W + objLoc[0])*0.5
#         # H = (H + objLoc[1]) * 0.5
#         W = objLoc[0]
#         H = objLoc[1]
#
#         # execute grasp
#         Gp.graspExecute(limb, gripper, W, H, Ang)
#         break
