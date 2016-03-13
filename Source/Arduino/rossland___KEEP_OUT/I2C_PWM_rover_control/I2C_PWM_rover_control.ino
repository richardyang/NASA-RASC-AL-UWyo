#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <avr/pgmspace.h>             // Enable use of PROGMEM
#include <ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include <Wire.h>                     // For I2C
#include <Adafruit_PWMServoDriver.h>  // For I2C PWM board


/* Testing Commands:
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<I2C_INDEX>, <servo_1>, etc.]}' */
*/

// PWM constants
const PROGMEM int PWM_FREQUENCY = 50;
const PROGMEM int PWM_RESOLUTION = 4096;
// Arm servo constants - Hitec HS-785HB 
const PROGMEM int ARM_SERVO_MIN = 126;        // "-315 degrees", min pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_MAX = 504;        // "+315 degrees", max pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_NEUTRAL = 315;    // "0 degrees", center pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_FULL_TURN = 216;  // "360 degrees", one full rotation
// Steering servo constants
const PROGMEM int STEER_MIN_PWM = 200;  // Max steering pwm
const PROGMEM int STEER_MAX_PWM = 550;  // Min steering pwm
// DC motor constants
const PROGMEM int DRIVE_MIN_PWM = 200;  // Max DC motor pwm
const PROGMEM int SPEED_MAX_PWM = 550;  // Min DC motor pwm

// Pin constants
const PROGMEM int VACUUM_PIN = 7;


/* Servo and DC Motor reference
+--------+-----+-------+-------+---------------------------+
| `-..-` ‖ I2C |  MSG  | ARRAY | DESCRIPTION               |
| .-``-. ‖ PIN | INDEX | INDEX |                           |
+========+=====+=======+=======+===========================+
|        ‖  0  |   0   |   0   | Arm Base                  |
|  Arm   ‖  1  |   1   |   1   | Arm Shoulder              |
| Servos ‖  2  |   2   |   2   | Arm Elbow                 |
|        ‖  3  |   3   |   3   | Arm Wrist                 |
+--------+-----+-------+-------+---------------------------+
|  Drive ‖  4  |   0   |   0   | Rear Wheel                |
| Servos ‖  5  |   1   |   1   | Front Right Wheel         |
|        ‖  6  |   2   |   2   | Front Left Wheel          |
+--------+-----+-------+-------+---------------------------+
|  Mast  ‖  8  |   0   |   0   | Mast Servo                |
+--------+-----+-------+-------+---------------------------+           
|        ‖ 11  |   0   |   0   | Rear Wheel                |
|  Drive ‖ 12  |   1   |   1   | Side Right Wheels (BOTH)  |
|   DC   ‖ 13  |   2   |   2   | Side Left Wheels (BOTH)   |
| Motors ‖ 14  |   3   |   3   | Front Right Wheel         |
|        ‖ 15  |   4   |   4   | Front Left Wheels         |
+--------+-----+-------+-------+---------------------------+  */
// I2C pins
const PROGMEM int ARM_BASE_PIN = 0;
const PROGMEM int ARM_SHOULDER_PIN = 1;
const PROGMEM int ARM_ELBOW_PIN = 2;
const PROGMEM int ARM_WRIST_PIN = 3;  
const PROGMEM int DRIVE_MOTOR_REAR_PIN = 4;
const PROGMEM int DRIVE_MOTOR_FRONT_RIGHT_PIN = 5;
const PROGMEM int DRIVE_MOTOR_FRONT_LEFT_PIN = 6;
const PROGMEM int MAST_PIN = 8;
const PROGMEM int STEER_SERVO_REAR_PIN = 11;
const PROGMEM int STEER_SERVO_SIDE_RIGHT_PIN = 12;
const PROGMEM int STEER_SERVO_SIDE_LEFT_PIN = 13;
const PROGMEM int STEER_SERVO_FRONT_RIGHT_PIN = 14;
const PROGMEM int STEER_SERVO_FRONT_LEFT_PIN = 15;

// I2C PWM variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ROS node handle
ros::NodeHandle nh;


/***** arm_angle_to_pulse()
  Converts a servo angle (0 - 180) into a PWM pulse
  @INPUT int - new arm servo angle
  @RETURN double - pwm pulse

  PWM_pulse = <neurtal_pwm> + (angle/360) * <full_turn_pwm>     */
uint16_t arm_angle_to_pulse(int int_angle) {
  double angle = int_angle * 1.0;  // Typecast angle to double to prevent overflow
  uint16_t pulse = (uint16_t) (ARM_SERVO_NEUTRAL + ((angle*ARM_SERVO_FULL_TURN)/360));
  // Check if within range
  if (pulse < ARM_SERVO_MIN) {
    pulse = (uint16_t) ARM_SERVO_MIN;
  } else if (pulse > ARM_SERVO_MAX) { 
    pulse = (uint16_t) ARM_SERVO_MAX;
  }
  return pulse;
}

/***** manual_arm_cmd_update()
  Interpreter for manual arm servo messages.
    Updates A SINGLE SERVO.
  @INPUT std_msgs::UInt16MultiArray cmd_msg
    [0] - servo number
    [1] - new servo angle                                       */
void manual_arm_cmd_update(const std_msgs::Int16MultiArray& cmd_msg) {
  uint8_t servo_number = (uint8_t) cmd_msg.data[0];
  int16_t new_servo_angle = (int16_t) cmd_msg.data[1];

  // Vacuum pump toggle
  if (servo_number == 0) {
    servo_number = ARM_BASE_PIN;
  } else if (servo_number == 1) {
    servo_number = ARM_SHOULDER_PIN;
  } else if (servo_number == 2) {
    servo_number = ARM_ELBOW_PIN;
  } else if (servo_number == 3) {
    servo_number = ARM_WRIST_PIN;
  } else if (servo_number == 4) {
    if (new_servo_angle == 0) {
      digitalWrite(VACUUM_PIN, LOW);
    } else {
      digitalWrite(VACUUM_PIN, HIGH);
    }
    return;   // No angle to update... just return
  }

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
ros::Subscriber<std_msgs::Int16MultiArray> sub_arm_manual("arm_cmd_manual", manual_arm_cmd_update);
//ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_steer_manual("drive_steer_manual", manual_drive_servo_update);
//ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive_speed_manual("drive_speed_manual", manual_drive_motor_update);


void setup(){
  nh.initNode();    // Initialize ROS node handle

  nh.subscribe(sub_arm_manual);   // Subcribe to manual arm control topic
  //nh.subscribe(sub_drive_steer_manual);
  //nh.subscribe(sub_drive_speed_manual);

  // Initialize I2C PWM board
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);

  // Send arm servos to home position
  pwm.setPWM(0, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(1, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(2, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(3, 0, ARM_SERVO_NEUTRAL);

  // Initialize vacuum pump controls
  pinMode(VACUUM_PIN, OUTPUT);
  digitalWrite(VACUUM_PIN, LOW);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
                                                                                      