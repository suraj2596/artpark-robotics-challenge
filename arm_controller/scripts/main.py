#!/usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

# Publishers

# scissor 1,2, gripper, payload, spine, height moving

# trash item array
pub = rospy.Publisher('chatter', String, queue_size=10)

# Subscribers
# trash item array
rospy.Subscriber("/trash", String, callback)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
