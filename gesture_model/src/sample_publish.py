#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('input_command_publisher', anonymous=True)
    pub = rospy.Publisher('input_command', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # Read input command
        command = input("Enter input command (left/right/up/down/open/close): ")

        # Publish input command
        pub.publish(command)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass