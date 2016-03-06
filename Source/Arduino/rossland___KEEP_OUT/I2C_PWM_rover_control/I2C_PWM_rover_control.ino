#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// Main ROS library
#include <ros.h>
// For communication over ROS
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
// PWM control
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*###############################################################################################*/
//  --- TESTING AND DEBUG

/* Testing Commands:
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<I2C_INDEX>, <servo_1>, etc.]}' */

// Debugging publisher - publishes to "chatter"
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
void publish(String msg) {
  char const * temp = msg.c_str();
  str_msg.data = temp;
  chatter.publish(&str_msg);
}
/*###############################################################################################*/


// PWM constants
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 4096
// Arm servo constants - Hitec HS-785HB 
#define ARM_SERVO_MIN  126      // "-315 degrees", minimum pulse length count (out of 4096@50Hz)
#define ARM_SERVO_MAX  504      // "+315 degrees", maximum pulse length count (out of 4096@50Hz)
#define ARM_SERVO_NEUTRAL 315   // "0 degrees", zeroed pulse length count (out of 4096@50Hz)
#define ARM_SERVO_FULL_TURN 216 // "360 degrees", one full rotation pulse length delta (4096@50Hz)
// Steer servo constants
#define STEER_MIN_PWM 200
#define STEER_MAX_PWM 550
// Drive motor constants
#define DRIVE_MIN_PWM 200
#define SPEED_MAX_PWM 550

/* Servo and DC Motor reference
        || I2C |  MSG  | ARRAY | DESCRIPTION               |
        || PIN | INDEX | INDEX |                           |
========++=====+=======+=======+===========================+
        ||  0  |   0   |   0   | Arm Base                  |
  Arm   ||  1  |   1   |   1   | Arm Shoulder              |
 Servos ||  2  |   2   |   2   | Arm Elbow                 |
        ||  3  |   3   |   3   | Arm Wrist                 |
--------++-----+-------+-------+---------------------------+
  Drive ||     |       |       | Back Wheels (BOTH)        |
 Servos ||     |       |       | Front Wheel               |
--------++-----+-------+-------+---------------------------+
        ||     |       |       | Front Left wheel          |
  Drive ||     |       |       | Front Right wheel         |
   DC   ||     |       |       | Side Left Wheels (BOTH)   |
 Motors ||     |       |       | Side Right Wheels (BOTH)  |
        ||     |       |       | Rear Wheel                |
--------++-----+-------+-------+---------------------------+  */

// I2C PWM variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ROS node handle
ros::NodeHandle nh;



/***** arm_angle_to_pulse()
  Converts a servo angle (0 - 180) into a PWM pulse
  @INPUT int - new arm servo angle
  @RETURN double - pwm pulse 
*/
uint16_t arm_angle_to_pulse(int int_angle) {
  // Typecast angle to double to prevent overflow
  double angle = int_angle * 1.0;

  // PWM = <neurtal_pwm> + (angle/360) * <full_turn_pwm>
  uint16_t pulse = (uint16_t) (ARM_SERVO_NEUTRAL + ((angle*ARM_SERVO_FULL_TURN)/360));

  // Check if pulse is a sentinel, or in allowed range
  if (pulse < ARM_SERVO_MIN) {
    pulse = (uint16_t) ARM_SERVO_MIN;
  } else if (pulse > ARM_SERVO_MAX) { 
    pulse = (uint16_t) ARM_SERVO_MAX;
  }

  return pulse;
}

/***** manual_arm_servo_update()
  Interpreter for manual arm servo messages.
    Updates A SINGLE SERVO.
  @INPUT std_msgs::UInt16MultiArray cmd_msg
    [0] - servo number
    [1] - new servo angle                       
*/
void manual_arm_servo_update(const std_msgs::Int16MultiArray& cmd_msg) {
  // Get servo servo number and angle from cmd_msg
  uint8_t servo_number = (uint8_t) cmd_msg.data[0];
  int16_t new_servo_angle = (int16_t) cmd_msg.data[1];

  // Calculate and set pulse
  double pulse = arm_angle_to_pulse(new_servo_angle);
  pwm.setPWM(servo_number, 0, pulse);
}

/***** manual_drive_servo_update() ###
  Interpreter for manual steer servo messages.
    Updates A SINGLE SERVO.
  @INPUT std_msgs::UInt16MultiArray cmd_msg
    [0] - servo number
    [1] - new servo angle
*/
void manual_drive_servo_update(const std_msgs::Int16MultiArray& cmd_msg) {
  // Get servo servo number and angle from cmd_msg
  uint8_t servo_number = (uint8_t) cmd_msg.data[0];
  uint16_t new_servo_angle = (uint16_t) cmd_msg.data[1];

}

/***** manual_drive_motor_update() ###
  Interpreter for manual steer servo messages.
  Updates A SINGLE MOTOR.

  @INPUT std_msgs::UInt16MultiArray cmd_msg
    [0] - motor number
    [1] - new motor speed   */
void manual_drive_motor_update(const std_msgs::Int16MultiArray& cmd_msg) {

}



/***** Subscribe to the following rostopics:
   + arm_cmd_manual       - manually set arm servo angles
   + drive_steer_manual   - manually set steering servo angles
   + drive_speed_manual   - manually set motor speeds             */
ros::Subscriber<std_msgs::Int16MultiArray> sub_arm_manual("arm_cmd_manual", manual_arm_servo_update);
//ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_steer_manual("drive_steer_manual", manual_drive_servo_update);
//ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_speed_manual("drive_speed_manual", manual_drive_motor_update);


void setup(){
  // Initialize ROS node handle
  nh.initNode();

  // Initializ ROS subscribers and publishers
  nh.advertise(chatter);
  nh.subscribe(sub_arm_manual);
  //nh.subscribe(sub_drive_steer_manual);
  //nh.subscribe(sub_drive_speed_manual);

  // Initialize Adafruit I2C PWM board
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);

  // Initialize arm positions
  pwm.setPWM(0, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(1, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(2, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(3, 0, ARM_SERVO_NEUTRAL);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
