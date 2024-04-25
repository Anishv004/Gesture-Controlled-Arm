#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

// Define servo pins
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SERVO4_PIN 6

// Define servo objects
Servo servo1; // Base servo
Servo servo2; // left servo
Servo servo3; // right servo
Servo servo4; //Gripper

int left_angle=90;
int right_angle=70;
int gripper_angle=0;
int base_angle=90;


// // Define initial servo positions
// int pos1 = 90; // Initial position for servo1 (base)
// int pos2 = 90; // Initial position for servo2 (shoulder)
// int pos3 = 90; // Initial position for servo3 (elbow)

// Define threshold for servo movement
const int threshold = 180;

// Define ROS node handle
ros::NodeHandle nh;

// std_msgs::String str_msg;
// Define ROS subscriber
// ros::Publisher chatter("servo_position", &str_msg);

// Define callback function for input command
void commandCallback(const std_msgs::String& msg) {
  String command = msg.data;

  if (command == "left" && base_angle<=170) {
    moveServo(&servo1, 10); // Move servo1 (base) left
    base_angle+=10;
  } else if (command == "right" &&  base_angle>=10) {
    moveServo(&servo1, -10)  ;  // Move servo1 (base) right
    base_angle-=10;
  } else if (command == "up") {
    if(left_angle<=100){
      moveServo(&servo2, 10); // Move servo2 (shoulder) up
      left_angle+=10;
    }
    if(right_angle>=80){
      moveServo(&servo3, -10);
      right_angle-=10;
    }
  } else if (command == "down") {
    // moveServo(&servo2, 10);  // Move servo2 (shoulder) down
    if(left_angle>=80){
      moveServo(&servo2, -10); // Move servo2 (shoulder) up
      left_angle-=10;      
    }
    if(right_angle<=90){
      moveServo(&servo3, 10);
      right_angle+=10;
    }
  } else if (command == "open") {
    servo4.write(15);
  } else if (command == "close") {
    servo4.write(0);
  }
}


ros::Subscriber<std_msgs::String> sub("input_command", &commandCallback);

// Function to move servo
void moveServo(Servo *servo, int direction) {
  int newPos = servo->read() + direction; // Calculate new position

  // Ensure the new position is within the servo's threshold
  if (newPos >= 0 && newPos <= threshold) {
    servo->write(newPos); // Move servo to the new position

    // Publish the new position to the same topic
    // std_msgs::Float64 posMsg;
    // posMsg.data = newPos;
    // // servo->read()
    // chatter.publish(&posMsg);

  }
}


void setup() {
  // Initialize ROS
  nh.initNode();
  // nh.initNode("arduino_node");
  nh.subscribe(sub);
  Serial.begin(57600);
  // Attach servo pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Set initial servo positions
  servo1.write(90);
  servo2.write(90);
  servo3.write(80);
  servo4.write(0);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
}
