#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def command_callback(msg):
    rospy.loginfo("Received command: %s", msg.data)
    pub.publish(msg.data)

if __name__ == "__main__":
    rospy.init_node('command_subscriber', anonymous=True)
    rospy.Subscriber('/command', String, command_callback)
    pub = rospy.Publisher('/input_command', String, queue_size=10)
    rospy.spin()
