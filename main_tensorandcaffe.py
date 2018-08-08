import rospy
import graspObjectImageFunc as gi
import intera_interface
import GraspingHelperClass as Gp
import GestRecB
from NLP_class import NLP_class
# ======================================================================================
rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global GRIPPER
gripper = intera_interface.Gripper('right')
gripper.open()

pre_grasp_pos = [-1.7220224609375, -1.1679736328125, -0.138830078125, 1.015201171875, 0.00028515625,
                 1.7164111328125,
                 -1.8061787109375]
# print limb.joint_angles()
Gp.smooth_move(limb, pre_grasp_pos, 0.2)


# #
# #  NLP
# #
nlp = NLP_class()
nlp.operate()

# #
# #  NLP
# #


print "Types of fruits: 1, apple; 2, avocado; 3, banana; 5, orange; 7, peach"
type_of_fruit = raw_input("Enter the fruit you want: ")

#
# GESTURE RECOGNITION
#
safe_move_r2l = [-0.5714521484375, -1.39324609375, 0.3229423828125, 2.6920048828125, -0.2382578125,
                 0.1504306640625,
                 1.6642041015625]
inputDir = "examples/Sawyer_GesRec/input_dir"
outputDir = "examples/Sawyer_GesRec/output_dir"
ges = GestRecB.GestureRecognition()
side_of_user = ges.execute(inputDir, outputDir)

if side_of_user == 1:
    # left side of user
    pre_grasp_pos = [-1.7220224609375, -1.1679736328125, -0.138830078125, 1.015201171875, 0.00028515625,
                     1.7164111328125,
                     -1.8061787109375]
else:
    # right side of user
    pre_grasp_pos = [0.8807666015625, -1.1366826171875, 0.9580830078125, 0.99812890625, -0.3362646484375,
                     1.837322265625,
                     -1.4236279296875]
    Gp.smooth_move(limb, safe_move_r2l, 0.2)

Gp.smooth_move(limb, pre_grasp_pos, 0.2)

#
# GESTURE RECOGNITION
#

frame = Gp.take_picture(0, 30)

objLoc = Gp.detect_objects(int(type_of_fruit), frame)

# print objLoc

worldVec, hom_Mtrx_c_b, rot = Gp.pixelToWorld(objLoc[0], objLoc[1])

# print worldVec

moveJoint = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                 p_x=worldVec[0], p_y=worldVec[1] + 0.05, p_z=0.5,  # q_x=0, q_y=0, q_z=0, q_w=0)
                                 q_x=rot[0], q_y=rot[1], q_z=rot[2], q_w=rot[3])

Gp.smooth_move(limb, positions=moveJoint, move_speed=0.2)

frame = Gp.take_picture(0, 30)

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

Gp.graspExecute(limb, gripper, W, H, Ang, worldVec[0], worldVec[1])
# rospy.sleep(1)
drop_fruit_pos = [-0.402546875, -1.1622041015625, 0.3266787109375, 2.2412666015625, -0.301185546875,
                  0.469794921875,
                  -1.2894443359375]
Gp.smooth_move(limb, drop_fruit_pos, 0.2)
rospy.sleep(1)
gripper.open()
rospy.sleep(0.5)
Gp.smooth_move(limb, safe_move_r2l, 0.2)
rospy.sleep(0.5)


