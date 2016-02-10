#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"
#include <sensor_msgs/Range.h>

// Needed for Adafruit Motor Shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* Commands to run:
  $ roscore
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<servo_1>, <servo_1>, etc.]}' */

/*#############################################################
!!! MESSAGE FORMAT !!!
!!!   MEASSAGES ARE PASSED TO THE ARDUINO OVER "ROSTOPIC"
!!!    --> Messages are passed as a "UInt16MultiArray
!!!   THE INDEX OF THE MESSAGES IS AS DEFINED HERE     */
// ### ARM SERVOS ###
#define MSG_ARM_BASE 0;
#define MSG_ARM_SHOULDER 1;
#define MSG_ARM_ELBOW 2
#define MSG_ARM_WRIST 3;
// ### DRIVE SERVOS ###
#define MSG_DRIVE_SRV_FRONT 0;
#define MSG_DRIVE_SRV_REAR 1;
// ### DC MOTORS ###
#define MSG_DRIVE_DC_FRONT_L 0;
#define MSG_DRIVE_DC_FRONT_R 1;
#define MSG_DRIVE_DC_REAR 2;
//#############################################################

// ### Ros NodeHandle ###
ros::NodeHandle nh;

/* ### ARM SERVOS ###
NOTE: INDEX represents index in rostopic messages AND array positions
PIN | INDEX | DESCRIPTION
----+-------+-----------------
  6 |   0   | Arm Base
  9 |   1   | Arm Shoulder
 10 |   2   | Arm Elbow
 11 |   3   | Arm Wrist         */
#define INDEX_ARM_BASE 0;       // Base Index
#define PIN_ARM_BASE 6;         // Base Pin
#define INDEX_ARM_SHOULDER 1;   // Shoulder Index
#define PIN_ARM_SHOULDER 9;     // Shoulder Pin
#define INDEX_ARM_ELBOW 2;      // Elbox Index
#define PIN_ARM_ELBOW 10;       // Elbow Pin
#define INDEX_ARM_WRIST 3;      // Wrist Index
#define PIN_ARM_WRIST 11;       // Wrist Pin
Servo arm_servos[4];            // Arm servo Array

/* ### DRIVE SERVOS ###
NOTE: INDEX represents index in rostopic messages AND array positions
PIN | INDEX | DESCRIPTION
----+-------+-----------------
  3 |   0   | REAR Wheel
  5 |   1   | Front Wheels (BOTH)     */
#define INDEX_DRIVE_SRV_REAR 0;   // REAR drive servo Index
#define PIN_DRIVE_SRV_REAR 3;     // REAR drive servo Pin
#define INDEX_DRIVE_SRV_FRONT 1;  // Front drive servo Index
#define PIN_DRIVE_SRV_FRONT 5;    // Front drive servo Pin
Servo drive_servos[2];        // Drive servo Array

/* ### DC MOTORS ###
NOTE: INDEX represents index in rostopic messages
NOTE: These are pin numbers on the Adafruit Motor Shield
PIN | INDEX | DESCRIPTION
----+-------+-----------------
  1 |   0   | Front Left Motor
  2 |   1   | Front Right Motor
  4 |   2   | Rear Motor
*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor * front_left_motor   = AFMS.getMotor(1);
Adafruit_DCMotor * front_right_motor  = AFMS.getMotor(2);
Adafruit_DCMotor * rear_motor         = AFMS.getMotor(4);


//===========================================================================//
//=================== CODE STARTS HERE ======================================//
//===========================================================================//

/* ### Publisher() ###
  Publishes to the "chatter" rostopic
*/
ros::Publisher chatter("chatter", &str_msg);
void publish_to_chatter(String msg) {
  std_msgs::String str_msg;         // temp message string
  char const * temp = msg.c_str();  // convert message to chars
  str_msg.data = temp;              // put into string container
  chatter.publish(&str_msg);        // publish to rostopic
}

/* ### update_arm_servos() ###
  Updates arm servo positions.
*/
void update_arm_servos(const std_msgs::UInt16MultiArray&  cmd_msg) {
  arm_servos[INDEX_ARM_BASE].write(cmd_msg.data[MSG_ARM_BASE]);
  arm_servos[INDEX_ARM_SHOULDER].write(cmd_msg.data[MSG_ARM_SHOULDER]);
  arm_servos[INDEX_ARM_ELBOW].write(cmd_msg.data[MSG_ARM_ELBOW]);
  arm_servos[INDEX_ARM_WRIST].write(cmd_msg.data[MSG_ARM_WRIST]);
}

/* ### update_drive_servos()
  Updates drive wheel servo positions
*/
void update_drive_servos(const std_msgs::UInt16MultiArray& cmd_msg) {
	drive_servos[INDEX_DRIVE_SRV_REAR].write(cmd_msg.data[MSG_DRIVE_SRV_REAR]);
  drive_servos[INDEX_DRIVE_SRV_FRONT].write(cmd_msg.data[MSG_DRIVE_SRV_FRONT]);
}

void update_drive_motors(const std_msgs::UInt16MultiArray& cmd_msg) {
  front_left_motor -> setspeed(cmd_msg.data[MSG_DRIVE_DC_FRONT_L]);
  front_right_motor -> setspeed(cmd_msg.data[MSG_DRIVE_DC_FRONT_R]);
  rear_motor -> setspeed(cmd_msg.data[MSG_DRIVE_DC_REAR])
}

// Subscribe to rostopic "arm_cmd" and "drive_cmd"
ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("arm_servos", update_arm_servos);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_servos("drive_servos", update_drive_servos);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_motors("drive_motors", update_drive_motors)


/* ### setup() ###
  Initializ:
    - ROS Nodes
    - ROS Topic Publishers/Subscribers
    - Servos
    - Motors     */
void setup(){
  nh.initNode();                  // Initialize ROS
  nh.advertise(chatter);          // Advertise --- "chatter"
  nh.subscribe(sub_arm);          // Subscribe --- "sub_arm"
  nh.subscribe(sub_drive_servos); // Subscribe --- "sub_drive_servos"
  nh.subscribe(sub_drive_motors); // Subscribe --- "sub_drive_motors"

  // START ADAFRUIT MOTOR SHIELD SERVICE && ATTACH DRIVE MOTORS
  AFMS.begin();
  front_left_motor -> run(RELEASE);
  front_right_motor -> run(RELEASE);
  rear_motor -> run(RELEASE);

  // ATTACH ARM SERVOS
  arm_servos[INDEX_ARM_BASE].attach(PIN_ARM_BASE);
  arm_servos[INDEX_ARM_SHOULDER].attach(PIN_ARM_SHOULDER);
  arm_servos[INDEX_ARM_ELBOW].attach(PIN_ARM_SHOULDER);
  arm_servos[INDEX_ARM_WRIST].attach(PIN_ARM_WRIST);

  // ATTACH DRIVE SERVOS
  drive_servos[INDEX_DRIVE_SRV_REAR].attach(PIN_DRIVE_SRV_REAR);
  drive_servos[INDEX_DRIVE_SRV_FRONT].attach(PIN_DRIVE_SRV_FRONT);
}

/* ### loop() ###
  Main program loop.
*/
void loop(){
  nh.spinOnce();
  delay(1);
}
