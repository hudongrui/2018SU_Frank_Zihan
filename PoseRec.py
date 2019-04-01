#!/usr/bin/python
import roslib
import rospy
import math
import tf
import pose_receiver

class PoseRecognition:

    def __init__(self):
        rospy.init_node('pose_receiver')
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self.count = 0
        self.c_thres = 30
        print "Finished init"

    def getTranslation(self, name, listener):
        user_counter = 1
        while True:
                tmp_head_frame_id = '/' + name + '_' + str(user_counter)
                try:
                    if user_counter == 10: # In case the number would blow up we limit down to 10
                        user_counter = 1
                    (trans,rot) = listener.lookupTransform('/camera_link', tmp_head_frame_id, rospy.Time(0)) 
                    # print "Found user " + str(user_counter)
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    user_counter += 1
                    continue
        return trans

    def getPose(self, head, left, right): # We inverted the direction because camera's mirror viewing
        r_dist = abs(head[1]-left[1]) * 100
        l_dist = abs(head[1]-right[1]) * 100

        if r_dist <= 40 and l_dist <= 40:
            # return "Unknown Direction"
            return 0

        elif l_dist > r_dist:
            if head[1] - right[1] > 0:
                # return "Pointing Left"
                return -1
            else:
                # return "Pointing Right" 
                return 1

        elif l_dist < r_dist:
            if head[1] - left[1] > 0:
                # return "Pointing Left"
                return -1
            else:
                # return "Pointing Right"
                return 1 
        else:
            # return "Unknown Direction"
            return 0

    def execute(self):
        while not rospy.is_shutdown():
            trans_head = self.getTranslation('head', self.listener)
            trans_l_hand = self.getTranslation('left_hand', self.listener)
            trans_r_hand = self.getTranslation('right_hand', self.listener)

            self.count += self.getPose(trans_head, trans_l_hand, trans_r_hand)
            
            # print getPose(trans_head, trans_l_hand, trans_r_hand)
            # print self.count
            if abs(self.count) == self.c_thres:
                break
            self.rate.sleep()
            print self.count
        if self.count < 0:
            return 1 # Left
        else:
            return 0 # right

if __name__ == '__main__':

    side_of_user =pose_receiver.execute()

    print side_of_user
    # return side_of_user
