import rospy
from std_msgs.msg import Int8 as int


rospy.init_node("blah")


def publisher():
    pub = rospy.Publisher('samplePublisher', int)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # print '.'
            pub.publish()

            rate.sleep()
            break


