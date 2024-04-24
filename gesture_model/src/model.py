#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time
import math

# Initialize MediaPipe Hands model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Initialize CvBridge
bridge = CvBridge()

# ROS Publisher
pub = rospy.Publisher('/hand_direction', Image, queue_size=10)

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

# Main function for hand detection and direction prediction
def main():
    # Initialize ROS node
    rospy.init_node('hand_direction_publisher', anonymous=True)

    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Get the screen dimensions
    screen_width = int(cap.get(3))
    screen_height = int(cap.get(4))
    screen_center = (screen_width // 2, screen_height // 2)

    # Variables for delay in detection
    last_detection_time = 0
    detection_interval = 2  # 2 seconds

    while not rospy.is_shutdown():
        # Read frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

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

        # Publish the frame
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        # Check if it's time to detect again
        current_time = time.time()
        if current_time - last_detection_time >= detection_interval:
            last_detection_time = current_time

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
