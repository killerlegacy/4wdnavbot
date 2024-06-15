#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

// Define ROS node handle
ros::NodeHandle nh;

// Define publishers for Twist and encoder readings
geometry_msgs::Twist cmd_vel_msg;
ros::Publisher cmd_vel_pub("/cmd_vel", &cmd_vel_msg);

std_msgs::Int32 encoder_msg_A;
ros::Publisher encoder_pub_A("/left_ticks", &encoder_msg_A);

std_msgs::Int32 encoder_msg_B;
ros::Publisher encoder_pub_B("/right_ticks", &encoder_msg_B);

// Motor control pins
const int motorA1 = 8;  // IN1 on L298N
const int motorA2 = 9;  // IN2 on L298N
const int enA = 10;     // ENA on L298N

const int motorB1 = 11; // IN3 on L298N
const int motorB2 = 12; // IN4 on L298N
const int enB = 13;     // ENB on L298N

// Encoder pins
const int encoderPinA1 = 20;
const int encoderPinA2 = 21;
const int encoderPinB1 = 18;
const int encoderPinB2 = 19;

// Variables
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Map Twist message to motor commands
  float linearVel = msg.linear.x;    // Forward/backward velocity
  float angularVel = msg.angular.z;  // Rotation velocity

  // Convert Twist message to motor commands
  float leftMotorSpeed = 255 * (linearVel - angularVel);
  float rightMotorSpeed = 255 * (linearVel + angularVel);

  // Ensure motor speeds are within range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Update motor speeds for left motor (Reversed logic)
  if (leftMotorSpeed >= 0) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  analogWrite(enA, abs(leftMotorSpeed));

  // Update motor speeds for right motor (Reversed logic)
  if (rightMotorSpeed >= 0) {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }
  analogWrite(enB, abs(rightMotorSpeed));
}

void encoderA_ISR() {
  // Increment encoder count for motor A on each pulse
  if (digitalRead(encoderPinA2) == HIGH) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

void encoderB_ISR() {
  // Increment encoder count for motor B on each pulse
  if (digitalRead(encoderPinB2) == HIGH) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVelCallback);

void setup() {
  Serial.begin(9600);

  // Initialize ROS node
  nh.initNode();

  // Advertise publishers
  nh.advertise(cmd_vel_pub);
  nh.advertise(encoder_pub_A);
  nh.advertise(encoder_pub_B);
  nh.subscribe(sub);

  // Initialize motor control pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enB, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), encoderB_ISR, CHANGE);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();

  // Publish encoder readings
  encoder_msg_A.data = encoderCountA;
  encoder_pub_A.publish(&encoder_msg_A);

  encoder_msg_B.data = encoderCountB;
  encoder_pub_B.publish(&encoder_msg_B);

  // Delay to prevent overloading the Arduino
  delay(10);
}
