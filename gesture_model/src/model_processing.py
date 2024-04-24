#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import math
from std_msgs.msg import String

# Initialize MediaPipe Hands model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Initialize CvBridge
bridge = CvBridge()

'''
# Function to get the direction based on hand position relative to the center
def get_direction(hand_landmarks, screen_center, screen_width, screen_height):
    if hand_landmarks is not None:
        hand_center = (
            int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * screen_width),
            int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * screen_height)
        )

        # Calculate distance between hand center and screen center
        distance = math.sqrt((hand_center[0] - screen_center[0])**2 + (hand_center[1] - screen_center[1])**2)
        center_threshold = min(screen_width, screen_height) * 0.1  # Adjust this threshold as needed

        if distance < center_threshold:
            return "centre"
        else:
            x_diff = hand_center[0] - screen_center[0]
            y_diff = hand_center[1] - screen_center[1]

            if abs(x_diff) > abs(y_diff):
                if x_diff > 0:
                    return "left"
                else:
                    return "right"
            else:
                if y_diff > 0:
                    return "bottom"
                else:
                    return "top"
    else:
        return "No hand detected"
'''

# Function to get the direction and gestures based on hand position and finger count
def get_direction(hand_landmarks, screen_center, screen_width, screen_height):
    if hand_landmarks is not None:
        # Count the number of fingers raised
        num_fingers = 0
        for finger in [mp_hands.HandLandmark.THUMB_TIP, mp_hands.HandLandmark.INDEX_FINGER_TIP,
                       mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_TIP,
                       mp_hands.HandLandmark.PINKY_TIP]:
            if hand_landmarks.landmark[finger].y < hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y:
                num_fingers += 1

        # Detect gestures based on finger count
        if num_fingers == 1:
            pub.publish('close')
            return "close"
        elif num_fingers == 2:
            pub.publish('open')
            return "open"
        else:
            hand_center = (
                int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * screen_width),
                int(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * screen_height)
            )

            # Calculate distance between hand center and screen center
            distance = math.sqrt((hand_center[0] - screen_center[0])**2 + (hand_center[1] - screen_center[1])**2)
            center_threshold = min(screen_width, screen_height) * 0.1  # Adjust this threshold as needed

            if distance < center_threshold:
                pub.publish('centre')
                return "centre"
            else:
                x_diff = hand_center[0] - screen_center[0]
                y_diff = hand_center[1] - screen_center[1]

                if abs(x_diff) > abs(y_diff):
                    if x_diff > 0:
                        pub.publish('left')
                        return "left"
                    else:
                        pub.publish('right')
                        return "right"
                else:
                    if y_diff > 0:
                        pub.publish('down')
                        return "down"
                    else:
                        pub.publish('top')
                        return "top"
    else:
        return "No hand detected"


# ROS Subscriber callback function
def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Convert frame to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect hands in the frame
    results = hands.process(rgb_frame)

    # Get hand landmarks and direction
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            direction = get_direction(hand_landmarks, screen_center, screen_width, screen_height)
            rospy.loginfo(direction)

            # Calculate bounding box coordinates
            x_min, y_min = screen_width, screen_height
            x_max, y_max = 0, 0
            for landmark in hand_landmarks.landmark:
                x, y = int(landmark.x * screen_width), int(landmark.y * screen_height)
                if x < x_min:
                    x_min = x
                if x > x_max:
                    x_max = x
                if y < y_min:
                    y_min = y
                if y > y_max:
                    y_max = y

            # Draw a green box around the detected hand
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Hand Detection", frame)
    cv2.waitKey(1)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('hand_direction_subscriber', anonymous=True)

    # Get the screen dimensions
    screen_width = rospy.get_param("~screen_width", 640)
    screen_height = rospy.get_param("~screen_height", 480)
    screen_center = (screen_width // 2, screen_height // 2)

    # ROS Subscriber
    rospy.Subscriber('/webcam_image', Image, image_callback)

    pub=rospy.Publisher('/commands',String,queue_size=5)

    # Spin
    rospy.spin()
