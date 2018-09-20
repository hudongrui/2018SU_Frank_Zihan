import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp
import copy
import interceptHelper as iH

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

drop_off_location = iH.drop_destinations()[0]

rospy.sleep(1)
# Pre-grasping joint angles
movingLoc = drop_off_location
pre_moving_loc = copy.deepcopy(drop_off_location)
pre_moving_loc[2] += 0.3
dQ = Gp.euler_to_quaternion(z=pre_moving_loc[3])
pre_drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                          p_x=pre_moving_loc[0], p_y=pre_moving_loc[1], p_z=pre_moving_loc[2],
                                          q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                      p_x=movingLoc[0], p_y=movingLoc[1], p_z=movingLoc[2],
                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])
Gp.smooth_move(limb, pre_drop_block_pos, 0.2)
rospy.sleep(1)
Gp.smooth_move(limb, drop_block_pos, 0.2)
rospy.sleep(1)

#####################################################################################################################
# Here is to take a picture at pre-grasping postion
frame = Gp.take_picture(0, 30)
cv2.imshow("Displaying Image", frame)
cv2.waitKey()
cv2.destroyAllWindows()
cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/Background.jpg", frame)
######################################################################################################################
