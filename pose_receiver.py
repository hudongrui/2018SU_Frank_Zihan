#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import sys

def getTranslation(name, listener):
	user_counter = 1
	while True:
			if user_counter == 10: # In case the number would blow up we limit down to 10
					user_counter = 1
			tmp_head_frame_id = '/' + name + '_' + str(user_counter)
			try:
				(trans,rot) = listener.lookupTransform('/camera_link', tmp_head_frame_id, rospy.Time(0)) 
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				user_counter += 1
				continue

	return trans


def getPose(head, left, right): # We inverted the direction because camera's mirror viewing
	r_dist = abs(head[1]-left[1]) * 100
	l_dist = abs(head[1]-right[1]) * 100
	# print l_dist 
	# print r_dist
	# left = -1
	# right = 1
	# unknow = 0

	if r_dist <= 40 and l_dist <= 40:
		#countL = 0;
		#countR = 0;
		# return "Unknown Direction"
		return 0

	elif l_dist > r_dist:
		if head[1] - right[1] > 0:
			#countL++
			#if countR!=0:
			#	countL = 0
			#	countR = 0
			#elif countL==30:
			#	return "left"
			# return "Pointing Left"
			return -1
		else:
			#countR++
			#if countL!=0:
			#	countL = 0
			#	countR = 0
			#elif countR==30:
			#	return "right"
			# return "Pointing Right" 
			return 1

	elif l_dist < r_dist:
		if head[1] - left[1] > 0:
			#countL++
			#if countR!=0:
			#	countL = 0
			#	countR = 0
			#elif countL==30:
			#	return "left"
			# return "Pointing Left"
			return -1
		else:
			#countR++
			#if countL!=0:
			#	countL = 0
			#	countR = 0
			#elif countR==30:
			#	return "right"
			# return "Pointing Right"
			return 1 

	else:
		#countL = 0;
		#countR = 0;
		# return "Unknown Direction"
		return 0


if __name__ == '__main__':
	rospy.init_node('pose_receiver')
	listener = tf.TransformListener()
	#lrQueue = queue.Queue(maxsize = 50);
	#countL = 0;
	#countR = 0;
	count = 0
	c_threshold = 10
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():

		trans_head = getTranslation('head', listener)
		trans_l_hand = getTranslation('left_hand', listener)
		trans_r_hand = getTranslation('right_hand', listener)

		# print("Head: " + str(trans_head))
		# print("Left Hand: " + str(trans_l_hand))
		# print("Right Hand: " + str(trans_r_hand))
		# print
		# print getPose(trans_head, trans_l_hand, trans_r_hand, count)
		count += getPose(trans_head, trans_l_hand, trans_r_hand)
		print count
		# print getPose(trans_head, trans_l_hand, trans_r_hand)
		if abs(count) == c_threshold:
			break
		rate.sleep()
	final_str = None
	if count < 0:
		final_str = 1
	else:
		final_str = 0

	with open('pose_rec_finish.txt', 'wb') as fh:
		fh.write(str(final_str))

	print "Gesture Rec Complete! =========================="

