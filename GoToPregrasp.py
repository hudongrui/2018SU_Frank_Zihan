import subprocess
import rospy
import intera_interface
import intera_interface.head_display as head
import cv2
import GraspingHelperClass as Gp

rospy.init_node("GraspingDemo")
global limb
limb = intera_interface.Limb('right')
global gripper
gripper = intera_interface.Gripper('right_gripper')
gripper.open()
headDisplay = head.HeadDisplay()

rospy.sleep(1)
# Pre-grasping joint angles
pre_grasp_pos = [-1.630677734375, -0.559880859375, -0.5919228515625, 0.723537109375, 0.4400439453125, 1.5005537109375,
                 1.35516796875]
# print limb.joint_angles()
dQ = [-0.7218173645115756, -0.6913802266664445, 0.014645313419448478, 0.027542499143427157]

drop_block_pos = Gp.ik_service_client(limb='right', use_advanced_options=True,
                                      p_x=0.2, p_y=-0.7869, p_z=0.4432,
                                      q_x=dQ[0], q_y=dQ[1], q_z=dQ[2], q_w=dQ[3])

Gp.move(limb, drop_block_pos, 0.2)
rospy.sleep(1)

#####################################################################################################################
# Here is to take a picture at pre-grasping postion
frame = Gp.take_picture(0, 30)
cv2.imshow("Displaying Image", frame)
cv2.waitKey()
cv2.destroyAllWindows()
cv2.imwrite("/home/team18/Frank_Ray_Zihan/2018SU_Frank_Zihan/TestPictures/With_Flash.jpg", frame)
######################################################################################################################
