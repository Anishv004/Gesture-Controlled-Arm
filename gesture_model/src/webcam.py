#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    # Initialize ROS node
    rospy.init_node('webcam_publisher', anonymous=True)

    # ROS Publisher
    pub = rospy.Publisher('/webcam_image', Image, queue_size=10)

    # Initialize CvBridge
    bridge = CvBridge()

    # Open the webcam
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # Read frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        # Publish the frame
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam
    cap.release()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
