#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <avr/pgmspace.h>             // Enable use of PROGMEM
#include <Wire.h>                     // For I2C PWM board
#include <Adafruit_PWMServoDriver.h>
#include <pt.h>                       // Protothread library
#include <ros.h>                      // ROS libraries
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"


/*----------    T E S T I N G   C O M M A N D S    ----------
To test this code without running the "Mission Control" code:
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<I2C_INDEX>, <servo_1>, etc.]}' */


/*-----------------------------------------------------------------------------------
//------------------    P I N   R E F E R E N C E   T A B L E    --------------------
//-----------------------------------------------------------------------------------
# Servo and DC Motor reference
+--------+-----+-------+-------+-------+--------------------------+
| \\  // ‖ I2C |  MSG  | ARRAY | MOTOR | DESCRIPTION              |
|   ||   ‖ PIN | INDEX | INDEX | ACTV8 |                          |
| //  \\ ‖     |       |       |  PIN  |                          |
+========+=====+=======+=======+=======+==========================+
|        ‖  0  |   0   |   0   |  N/A  | Arm Base                 |
|  Arm   ‖  1  |   1   |   1   |  N/A  | Arm Shoulder             |
| Servos ‖  2  |   2   |   2   |  N/A  | Arm Elbow                |
|        ‖  3  |   3   |   3   |  N/A  | Arm Wrist                |
+--------+-----+-------+-------+-------+--------------------------+
|  Drive ‖  4  |   0   |   0   |  N/A  | Rear Wheel               |
| Servos ‖  5  |   1   |   1   |  N/A  | Front Right Wheel        |
|        ‖  6  |   2   |   2   |  N/A  | Front Left Wheel         |
+--------+-----+-------+-------+-------+--------------------------+
|  Mast  ‖  8  |   0   |   0   |  N/A  | Mast Servo               |
+--------+-----+-------+-------+-------+--------------------------+
|        ‖ 11  |   0   |   0   |   2   | Rear Wheel               |
|  Drive ‖ 12  |   1   |   1   |   4   | Side Right Wheels (BOTH) |
|   DC   ‖ 13  |   2   |   2   |   7   | Side Left Wheels (BOTH)  |
| Motors ‖ 14  |   3   |   3   |   8   | Front Right Wheel        |
|        ‖ 15  |   4   |   4   |  12   | Front Left Wheels        |
+--------+-----+-------+-------+-------+--------------------------+                */


//-----------------------------------------------------------------------------------
//------------------------------   C O N S T A N T S   ------------------------------
//-----------------------------------------------------------------------------------
//----------   P W M    C O N S T A N T S   ----------
const PROGMEM int PWM_FREQUENCY      = 50;
const PROGMEM int PWM_RESOLUTION     = 4096;
const PROGMEM uint16_t MAX_PWM_VALUE = 4093;  // avoid extremes
const PROGMEM uint16_t MIN_PWM_VALUE = 3;     // avoid extremes

//----------   A R M   S E R V O   C O N S T A N T S   ----------
// # Hitec HS-785HB 
const PROGMEM int ARM_SERVO_MIN       = 126;  // "-315 degrees", min pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_MAX       = 504;  // "+315 degrees", max pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_NEUTRAL   = 315;  // "0 degrees", center pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_SERVO_FULL_TURN = 216;  // "360 degrees", one full rotation

//----------   S T E E R I N G   S E R V O   C O N S T A N T S   ----------
const PROGMEM int STEER_MIN       = 200;
const PROGMEM int STEER_MAX       = 500; 
const PROGMEM int STEER_NEUTRAL   = 400;
const PROGMEM int STEER_FULL_TURN = 200;

//----------   D C   M O T O R   C O N S T A N T S   ----------
// # If the motor pwms are too close to neutral (4096/2 +- ~5%)
const PROGMEM int MIN_FORWARD_SPEED_PWM = 2200; // Max DC motor pwm
const PROGMEM int MIN_REVERSE_SPEED_PWM = 1992; // Min DC motor pwm
const PROGMEM int STATIONARY_PWM        = 2048; // Stopped DC motor pwm


//-----------------------------------------------------------------------------------
//------------------------   P I N S   C O N S T A N T S   --------------------------
//-----------------------------------------------------------------------------------
//----------    A R M   P I N S    ----------
const PROGMEM int ARM_PWM_PIN_BASE = 0;
const PROGMEM int ARM_PWM_PIN_SHOULDER = 1;
const PROGMEM int ARM_PWM_PIN_ELBOW = 2;
const PROGMEM int ARM_PWM_PIN_WRIST = 3;  

//----------    S T E E R   P I N S    ----------
const PROGMEM int STEER_PWM_PIN_REAR = 4;
const PROGMEM int STEER_PWM_PIN_FRONT_RIGHT = 5;
const PROGMEM int STEER_PWM_PIN_FRONT_LEFT = 6;

//----------    D R I V E   P I N S    ----------
const PROGMEM int DRIVE_PWM_PIN_REAR = 11;
const PROGMEM int DRIVE_PWM_PIN_SIDE_RIGHT = 12;
const PROGMEM int DRIVE_PWM_PIN_SIDE_LEFT = 13;
const PROGMEM int DRIVE_PWM_PIN_FRONT_RIGHT = 14;
const PROGMEM int DRIVE_PWM_PIN_FRONT_LEFT = 15;

//----------    A T M E G A   G R I P P E R   P I N     ----------
const PROGMEM int GRIPPER_VACUUM_PIN = 7;

//----------    A T M E G A   M O T O R   A C T I V A T E   P I N S     ----------
const PROGMEM int DRIVE_ACTIVATE_PIN_REAR = 2;
const PROGMEM int DRIVE_ACTIVATE_PIN_SIDE_RIGHT = 4;
const PROGMEM int DRIVE_ACTIVATE_PIN_SIDE_LEFT = 7;
const PROGMEM int DRIVE_ACTIVATE_PIN_FRONT_RIGHT = 8;
const PROGMEM int DRIVE_ACTIVATE_PIN_FRONT_LEFT = 12;


//-----------------------------------------------------------------------------------
//--------------   A R R A Y   &   M E S S A G E   C O S N T A N T S   --------------
//-----------------------------------------------------------------------------------
//----------    D R I V E   M E S S A G E   I N D E C E S    ----------
const PROGMEM int DRIVE_MSG_INDEX_REAR = 0;
const PROGMEM int DRIVE_MSG_INDEX_SIDE_RIGHT = 1;
const PROGMEM int DRIVE_MSG_INDEX_SIDE_LEFT = 2;
const PROGMEM int DRIVE_MSG_INDEX_FRONT_RIGHT = 3;
const PROGMEM int DRIVE_MSG_INDEX_FRONT_LEFT = 4;

//----------    D R I V E   A R R A Y   I N D E C E S    ----------
const PROGMEM int DRIVE_ARRAY_INDEX_REAR = 0;
const PROGMEM int DRIVE_ARRAY_INDEX_SIDE_RIGHT = 1;
const PROGMEM int DRIVE_ARRAY_INDEX_SIDE_LEFT = 2;
const PROGMEM int DRIVE_ARRAY_INDEX_FRONT_RIGHT = 3;
const PROGMEM int DRIVE_ARRAY_INDEX_FRONT_LEFT = 4;


//-----------------------------------------------------------------------------------
//-----------------------   G L O B A L   V A R I A B L E S   -----------------------
//-----------------------------------------------------------------------------------
//----------    M O T O R   A C C E L E R A T I O N    ----------
static struct pt motor_protothread;         // Protothread for motor speed updates
const PROGMEM int MOTOR_PTHREAD_DELAY = 20; // Motor speed update delay (in ms)
uint16_t TARGET_MOTOR_PWM[5];               // Target pulse for DC motors
uint16_t CURRENT_MOTOR_PWM[5];              // Target pulse for DC motors

//----------    O T H E R   V A R I A B L E S    ----------
// ROS node handle (makes everything work)
ros::NodeHandle nh;
// Adafruit PWM Driver (for driving servos and motors)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



//-----------------------------------------------------------------------------------
//--------------------   C O D E   B E G I N S   H E R E   --------------------------
//-----------------------------------------------------------------------------------

/***** Converting angles to PWM pulses
  These functions convert a servo angles (in degrees) into a PWM pulses
  Both arm_angle_to_pulse() and drive_angle_to_pulse() simply call angle_to_pulse()
    with the appropriate values

  Here's some math:
    PWM_pulse = <neurtal_pwm> + (angle/360) * <full_turn_pwm>     
*/
uint16_t angle_to_pulse(int int_angle, 
                        int servo_neutral, int servo_full_turn, 
                        int servo_min, int servo_max) {
  // Typecast angle to double to prevent overflow
  double angle = int_angle * 1.0;
  uint16_t pulse = (uint16_t) (servo_neutral + ((angle*servo_full_turn)/360));
  // Check if within range
  if (pulse < servo_min) { 
    pulse = (uint16_t) servo_min; 
  } else if (pulse > ARM_SERVO_MAX) {
    pulse = (uint16_t) servo_max;
  }
  return pulse;
}
uint16_t arm_angle_to_pulse(int int_angle) {
  return angle_to_pulse(int_angle, ARM_SERVO_NEUTRAL, 
    ARM_SERVO_FULL_TURN, ARM_SERVO_MIN, ARM_SERVO_MAX);
}
uint16_t drive_angle_to_pulse(int int_angle) {
  return angle_to_pulse(int_angle, ARM_SERVO_NEUTRAL, 
    ARM_SERVO_FULL_TURN, ARM_SERVO_MIN, ARM_SERVO_MAX);
}


/***** manual_arm_cmd_callback()
  Callback function for updating arm servos.
*/
void manual_arm_cmd_callback(const std_msgs::Int16MultiArray& cmd_msg) {

  
  uint8_t servo_number = (uint8_t) cmd_msg.data[0];
  int16_t new_servo_angle = (int16_t) cmd_msg.data[1];

  // Vacuum pump toggle
  if (servo_number == 0) {
    servo_number = ARM_PWM_PIN_BASE;
  } else if (servo_number == 1) {
    servo_number = ARM_PWM_PIN_SHOULDER;
  } else if (servo_number == 2) {
    servo_number = ARM_PWM_PIN_ELBOW;
  } else if (servo_number == 3) {
    servo_number = ARM_PWM_PIN_WRIST;
  } else if (servo_number == 4) {
    if (new_servo_angle == 0) {
      digitalWrite(GRIPPER_VACUUM_PIN, LOW);
    } else {
      digitalWrite(GRIPPER_VACUUM_PIN, HIGH);
    }
    return;   // No angle to update... just return
  }

  // Calculate and set pulse
  uint16_t pulse = arm_angle_to_pulse(new_servo_angle);
  pwm.setPWM(servo_number, 0, pulse);
}

/***** manual_drive_servo_callback() ###
  Callback function for updating drive servos.
*/
void manual_drive_servo_callback(const std_msgs::Int16MultiArray& cmdMsg) {

  uint16_t pulse;
  int16_t newAngle;
  // Update rear steer servo
  newAngle = (int16_t) cmdMsg.data[0];
  pulse = drive_angle_to_pulse(newAngle);
  pwm.setPWM(STEER_PWM_PIN_REAR, 0, pulse);

  // Update front right steer servo
  newAngle = (int16_t) cmdMsg.data[1];
  pulse = drive_angle_to_pulse(newAngle);
  pwm.setPWM(STEER_PWM_PIN_FRONT_RIGHT, 0, pulse);

  // Update front left steer servo
  newAngle = (int16_t) cmdMsg.data[2];
  pulse = drive_angle_to_pulse(newAngle);
  pwm.setPWM(STEER_PWM_PIN_FRONT_LEFT, 0, pulse);
}


/***** Converting angles to PWM pulses
  These functions convert a servo angles (in degrees) into a PWM pulses
  Both arm_angle_to_pulse() and drive_angle_to_pulse() simply call angle_to_pulse()
    with the appropriate values

  Here's some math:
    PWM_pulse = <neurtal_pwm> + (angle/360) * <full_turn_pwm>  
*/
void pthread_motor_helper(int motor_index, int drive_pwm_pin, int drive_activate_pin) {
  /* 
    0 < MIN_REVERSE_SPEED < STATIONARY_PWM < MIN_FORWARD_SPEED < 4096

    Cases:
      if TARGET > CURRENT
        if CURRENT == STATIONARY_PWM then CURRENT = MIN_FORWARD_SPEED
        else increment CURRENT
      else if TARGET < CURRENT
        if CURRENT == STATIONARY_PWM then CURRENT = MIN_REVERSE_SPEED
        else decrement CURRENT
      else return
  */
  if (TARGET_MOTOR_PWM[motor_index] > CURRENT_MOTOR_PWM[motor_index]) {
    if (CURRENT_MOTOR_PWM[motor_index] == STATIONARY_PWM ) {
      CURRENT_MOTOR_PWM[motor_index] = MIN_FORWARD_SPEED_PWM;
    } else {
      CURRENT_MOTOR_PWM[motor_index]++;
    }
  } else if (TARGET_MOTOR_PWM[motor_index] < CURRENT_MOTOR_PWM[motor_index]) {
    if (CURRENT_MOTOR_PWM[motor_index] == STATIONARY_PWM ) {
      CURRENT_MOTOR_PWM[motor_index] = MIN_REVERSE_SPEED_PWM;
    } else {
      CURRENT_MOTOR_PWM[motor_index]--;
    }
  } else {
    return;   // Exit if there is nothing to change
  }

  // Check if within range for activate pin ("STATIONARY_PWM" +- 5%)
  if (MIN_FORWARD_SPEED_PWM > CURRENT_MOTOR_PWM[motor_index] 
    && CURRENT_MOTOR_PWM[motor_index] > MIN_REVERSE_SPEED_PWM) {
    // Deactivate motor and set to "STATIONARY_PWM"
    //  (if the speed is too low, the H-BRIDGE MIGHT OVERHEAT)
    CURRENT_MOTOR_PWM[motor_index] = STATIONARY_PWM ;
    digitalWrite(drive_activate_pin, LOW);
  } else {
    digitalWrite(drive_activate_pin, HIGH);
  }
  // If the desired speed is too close "STATIONARY_PWM", 
  //  turn off activate pin and set speed to "STATIONARY_PWM"
  pwm.setPWM(drive_pwm_pin, 0, CURRENT_MOTOR_PWM[motor_index]);
}

static int pthread_update_DC_motors(struct pt * pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(true) {
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > MOTOR_PTHREAD_DELAY);
    timestamp = millis();
    pthread_motor_helper(DRIVE_ARRAY_INDEX_REAR,
                         DRIVE_PWM_PIN_REAR, 
                         DRIVE_ACTIVATE_PIN_REAR);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_SIDE_RIGHT,
                         DRIVE_PWM_PIN_SIDE_RIGHT, 
                         DRIVE_ACTIVATE_PIN_SIDE_RIGHT);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_SIDE_LEFT, 
                         DRIVE_PWM_PIN_SIDE_LEFT, 
                         DRIVE_ACTIVATE_PIN_SIDE_LEFT);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_FRONT_RIGHT, 
                         DRIVE_PWM_PIN_FRONT_RIGHT,
                         DRIVE_ACTIVATE_PIN_FRONT_RIGHT);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_FRONT_LEFT,
                         DRIVE_PWM_PIN_FRONT_LEFT,
                         DRIVE_ACTIVATE_PIN_FRONT_LEFT);
  }
  PT_END(pt);
}

/***** drive_speed_to_pulse() ###
  Converts a drive speed to a PWM pulse,
    checking if this pulse is within range
    and is not too close to "STATIONARY_PWM"

  !!! THIS FUNCTION SHOULD NEVER RETURN ANY VALUE 
  !!! WITHIN THE MIN FORWARD/BACKWARD PWMs
  !!! OTHER THAN "neutral" (ie STATIONARY_PWM )
*/
uint16_t drive_speed_to_pulse(int int_speed) {
  // Typecast angle to double to prevent overflow
  double speed = int_speed * 1.0 + STATIONARY_PWM ;
  uint16_t pulse = (uint16_t) speed;
  if (pulse > MAX_PWM_VALUE) { 
    pulse = MAX_PWM_VALUE;
  } else if (MIN_PWM_VALUE < 0) {
    pulse = MIN_PWM_VALUE;
  } else if (MIN_FORWARD_SPEED_PWM > pulse && pulse > MIN_REVERSE_SPEED_PWM) {
    // Too close to "STATIONARY_PWM"... set to "STATIONARY_PWM"
    pulse = STATIONARY_PWM;
  }
  return pulse;
}

/***** manual_drive_callback() ###
  Callback function for updating drive motors.
*/
void manual_drive_callback(const std_msgs::Int16MultiArray& cmd_msg) {


  uint16_t pulse;
  int16_t newSpeed;
  // Update rear drive motor
  newSpeed = (int16_t) cmd_msg.data[0];
  TARGET_MOTOR_PWM[0] = drive_speed_to_pulse(newSpeed);

  // Update side right drive motors
  newSpeed = (int16_t) cmd_msg.data[1];
  TARGET_MOTOR_PWM[1] = drive_speed_to_pulse(newSpeed);

  // Update side left drive motors
  newSpeed = (int16_t) cmd_msg.data[2];
  TARGET_MOTOR_PWM[2] = drive_speed_to_pulse(newSpeed);

  // Update front right drive motor
  newSpeed = (int16_t) cmd_msg.data[3];
  TARGET_MOTOR_PWM[3] = drive_speed_to_pulse(newSpeed);

  // Update front left drive motor
  newSpeed = (int16_t) cmd_msg.data[4];
  TARGET_MOTOR_PWM[4] = drive_speed_to_pulse(newSpeed);
}


/***** Subscribe to the following rostopics:
   + arm_cmd_manual       - manually set arm servo angles
   + drive_steer_manual   - manually set steering servo angles
   + drive_speed_manual   - manually set motor speeds             */
ros::Subscriber<std_msgs::Int16MultiArray> sub_arm_manual("arm_cmd_manual", manual_arm_cmd_callback);
ros::Subscriber<std_msgs::Int16MultiArray> sub_drive_steer_manual("steer_cmd_manual", manual_drive_servo_callback);
ros::Subscriber<std_msgs::Int16MultiArray> sub_drive_speed_manual("drive_cmd_manual", manual_drive_callback);


void setup(){
  nh.initNode();    // Initialize ROS node handle
  nh.subscribe(sub_arm_manual);         // Subcribe to manual arm control topic
  nh.subscribe(sub_drive_steer_manual); // Subcribe to manual steering control topic
  nh.subscribe(sub_drive_speed_manual); // Subcribe to manual speed control topic


  // Initialize I2C PWM board
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);

  // Initialize arm servos (set to home position)
  pwm.setPWM(0, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(1, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(2, 0, ARM_SERVO_NEUTRAL);
  pwm.setPWM(3, 0, ARM_SERVO_NEUTRAL);

  // Initialize vacuum pump controls
  pinMode(GRIPPER_VACUUM_PIN, OUTPUT);
  digitalWrite(GRIPPER_VACUUM_PIN, LOW);

  // Initialize array for "target" motor pwm (to "STATIONARY_PWM")
  TARGET_MOTOR_PWM[DRIVE_ARRAY_INDEX_REAR]         = STATIONARY_PWM ;
  TARGET_MOTOR_PWM[DRIVE_ARRAY_INDEX_SIDE_RIGHT]   = STATIONARY_PWM ;
  TARGET_MOTOR_PWM[DRIVE_ARRAY_INDEX_SIDE_LEFT]    = STATIONARY_PWM ;
  TARGET_MOTOR_PWM[DRIVE_ARRAY_INDEX_FRONT_RIGHT]  = STATIONARY_PWM ;
  TARGET_MOTOR_PWM[DRIVE_ARRAY_INDEX_FRONT_LEFT]   = STATIONARY_PWM ;

  // Initialize array for "current" motor pwm (to "STATIONARY_PWM")
  CURRENT_MOTOR_PWM[DRIVE_ARRAY_INDEX_REAR]        = STATIONARY_PWM ;
  CURRENT_MOTOR_PWM[DRIVE_ARRAY_INDEX_SIDE_RIGHT]  = STATIONARY_PWM ;
  CURRENT_MOTOR_PWM[DRIVE_ARRAY_INDEX_SIDE_LEFT]   = STATIONARY_PWM ;
  CURRENT_MOTOR_PWM[DRIVE_ARRAY_INDEX_FRONT_RIGHT] = STATIONARY_PWM ;
  CURRENT_MOTOR_PWM[DRIVE_ARRAY_INDEX_FRONT_LEFT]  = STATIONARY_PWM ;

  // Initialize motor "activate" pins
  pinMode(DRIVE_ACTIVATE_PIN_REAR, OUTPUT);
  pinMode(DRIVE_ACTIVATE_PIN_SIDE_RIGHT, OUTPUT);
  pinMode(DRIVE_ACTIVATE_PIN_SIDE_LEFT, OUTPUT);
  pinMode(DRIVE_ACTIVATE_PIN_FRONT_RIGHT, OUTPUT);
  pinMode(DRIVE_ACTIVATE_PIN_FRONT_LEFT, OUTPUT);
  digitalWrite(DRIVE_ACTIVATE_PIN_REAR, LOW);
  digitalWrite(DRIVE_ACTIVATE_PIN_SIDE_RIGHT, LOW);
  digitalWrite(DRIVE_ACTIVATE_PIN_SIDE_LEFT, LOW);
  digitalWrite(DRIVE_ACTIVATE_PIN_FRONT_RIGHT, LOW);
  digitalWrite(DRIVE_ACTIVATE_PIN_FRONT_LEFT, LOW);

  // Initialize protothread(s)
  PT_INIT(&motor_protothread);
}

void loop(){
  pthread_update_DC_motors(&motor_protothread);
  nh.spinOnce();
  delay(10);
}
                                                                                      