#include <avr/pgmspace.h>             // Enable use of PROGMEM
#include <Wire.h>                     // For I2C PWM board
#include <Adafruit_PWMServoDriver.h>
#include <pt.h>                       // Protothread library
#include <ros.h>                      // ROS libraries
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>

/*----------  R A M   U S A G E  ----------
  10% - Wire.h

  NEEDED FOR EXECUTION
  36 B - message array
   2 B - local int in callback

   OR 

   2 B - local int in motor update

*/

/*----------    T E S T I N G   C O M M A N D S    ----------
To test this code without running the "Mission Control" code:
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<I2C_INDEX>, <servo_1>, etc.]}' */


/*-----------------------------------------------------------------------------------
//------------------    P I N   R E F E R E N C E   T A B L E    --------------------
//-----------------------------------------------------------------------------------
# Servo and DC Motor reference
+---------+-----+-------+-------+--------------------------+
| \\  //  ‖ I2C |  MSG  | ARRAY | DESCRIPTION              |
|   ||    ‖ PIN | INDEX | INDEX |                          |
| //  \\  ‖     |       |       |                          |
+=========+=====+=======+=======+==========================+
|         ‖  0  |   0   |  N/A  | Arm Base                 |
|  Arm    ‖  1  |   1   |  N/A  | Arm Shoulder             |
| Servos  ‖  2  |   2   |  N/A  | Arm Elbow                |
|         ‖  3  |   3   |  N/A  | Arm Wrist                |
+---------+-----+-------+-------+--------------------------+
| Gripper ‖  8  |  12   |  N/A  | Gripper Rotation Servo   |
| Servos  ‖  9  |  13   |  N/A  | Gripper Claw Servo       |
+---------+-----+-------+-------+--------------------------+
|  Drive  ‖  4  |   4   |  N/A  | Rear Wheel               |
| Servos  ‖  5  |   5   |  N/A  | Front Right Wheel        |
|         ‖  6  |   6   |  N/A  | Front Left Wheel         |
+---------+-----+-------+-------+--------------------------+
|         ‖ 11  |   7   |   0   | Rear Wheel               |
|  Drive  ‖ 12  |   8   |   1   | Side Right Wheels (BOTH) |
|   DC    ‖ 13  |   9   |   2   | Side Left Wheels (BOTH)  |
| Motors  ‖ 14  |  10   |   3   | Front Right Wheel        |
|         ‖ 15  |  11   |   4   | Front Left Wheels        |
+---------+-----+-------+-------+--------------------------+  
|  Mast   ‖  7  |  14   |  N/A  | Mast Servo               |
+---------+-----+-------+-------+--------------------------+              */


//-----------------------------------------------------------------------------------
//------------------------------   C O N S T A N T S   ------------------------------
//-----------------------------------------------------------------------------------
//----------   P W M    C O N S T A N T S   ----------
const PROGMEM int PWM_FREQUENCY         = 50;
const PROGMEM int PWM_RESOLUTION        = 4096;
const PROGMEM uint16_t ABSOLUTE_MAX_PWM = 4093;  // avoid extremes
const PROGMEM uint16_t ABSOLUTE_MIN_PWM = 3;     // avoid extremes

//----------   A R M   S E R V O   C O N S T A N T S   ----------
// # Hitec HS-785HB 
const PROGMEM int ARM_PWM_MIN         = 126;  // "-315 degrees", min pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_PWM_MAX         = 504;  // "+315 degrees", max pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_PWM_NEUTRAL     = 315;  // "0 degrees", center pulse length count (out of 4096@50Hz)
const PROGMEM int ARM_PWM_360_DEGREES = 216;  // "360 degrees", one full rotation

//----------   S T E E R I N G   S E R V O   C O N S T A N T S   ----------
const PROGMEM int STEER_PWM_MIN         = 105;
const PROGMEM int STEER_PWM_MAX         = 495;
const PROGMEM int STEER_PWM_NEUTRAL     = 295;
const PROGMEM int STEER_PWM_360_DEGREES = 720;

//----------   G R I P P E R   S E R V O   C O N S T A N T S   ----------
// # Hitec HS-422 (gripper rotation servo)
const PROGMEM int GRIPPER_ROTATE_PWM_MIN         = 105;
const PROGMEM int GRIPPER_ROTATE_PWM_MAX         = 495;
const PROGMEM int GRIPPER_ROTATE_PWM_NEUTRAL     = 295;
const PROGMEM int GRIPPER_ROTATE_PWM_360_DEGREES = 720;
// # Hitec HS-322HD (gripper claw servo)
const PROGMEM int GRIPPER_CLAW_PWM_CLOSED = 276;
const PROGMEM int GRIPPER_CLAW_PWM_OPEN   = 355;

//----------   D C   M O T O R   C O N S T A N T S   ----------
const PROGMEM int NEUTRAL_SPEED_PWM     = 292; // Neutral speed PWM value


//-----------------------------------------------------------------------------------
//------------------------   P I N S   C O N S T A N T S   --------------------------
//-----------------------------------------------------------------------------------
//----------    A R M   P I N S    ----------
const PROGMEM int ARM_PWM_PIN_BASE     = 0;
const PROGMEM int ARM_PWM_PIN_SHOULDER = 1;
const PROGMEM int ARM_PWM_PIN_ELBOW    = 2;
const PROGMEM int ARM_PWM_PIN_WRIST    = 3;

//----------    S T E E R   P I N S    ----------
const PROGMEM int STEER_PWM_PIN_R   = 4;
const PROGMEM int STEER_PWM_PIN_F_R = 5;
const PROGMEM int STEER_PWM_PIN_F_L = 6;

//----------    G R I P P E R   P I N S    ----------
const PROGMEM int GRIPPER_PWM_PIN_ROTATE = 8;
const PROGMEM int GRIPPER_PWM_PIN_CLAW 	 = 9;

//----------    D R I V E   P I N S    ----------
const PROGMEM int DRIVE_PWM_PIN_R   = 11;
const PROGMEM int DRIVE_PWM_PIN_S_R = 12;
const PROGMEM int DRIVE_PWM_PIN_S_L = 13;
const PROGMEM int DRIVE_PWM_PIN_F_R = 14;
const PROGMEM int DRIVE_PWM_PIN_F_L = 15;

//----------    M A S T   P I N    ----------
const PROGMEM int MAST_PWM_PIN = 7;

//-----------------------------------------------------------------------------------
//--------------   A R R A Y   &   M E S S A G E   C O S N T A N T S   --------------
//-----------------------------------------------------------------------------------
//----------    M E S S A G E   I N D E C E S    ----------
const PROGMEM int MSG_INDEX_ARM_BASE       =  0;
const PROGMEM int MSG_INDEX_ARM_SHOULDER   =  1;
const PROGMEM int MSG_INDEX_ARM_ELBOW      =  2;
const PROGMEM int MSG_INDEX_ARM_WRIST      =  3;
const PROGMEM int MSG_INDEX_STEER_R        =  4;
const PROGMEM int MSG_INDEX_STEER_F_R      =  5;
const PROGMEM int MSG_INDEX_STEER_F_L      =  6;
const PROGMEM int MSG_INDEX_DRIVE_R        =  7;
const PROGMEM int MSG_INDEX_DRIVE_S_R      =  8;
const PROGMEM int MSG_INDEX_DRIVE_S_L      =  9;
const PROGMEM int MSG_INDEX_DRIVE_F_R      = 10;
const PROGMEM int MSG_INDEX_DRIVE_F_L      = 11;
const PROGMEM int MSG_INDEX_GRIPPER_ROTATE = 12;  
const PROGMEM int MSG_INDEX_GRIPPER_CLAW   = 13;  
//const PROGMEM int MSG_INDEX_MAST_STEPPER = 14;  // MIGHT NOT BE USED...

//----------    M O T O R   A R R A Y   I N D E C E S    ----------
const PROGMEM int DRIVE_ARRAY_INDEX_R   = 0;
const PROGMEM int DRIVE_ARRAY_INDEX_S_R = 1;
const PROGMEM int DRIVE_ARRAY_INDEX_S_L = 2;
const PROGMEM int DRIVE_ARRAY_INDEX_F_R = 3;
const PROGMEM int DRIVE_ARRAY_INDEX_F_L = 4;


//-----------------------------------------------------------------------------------
//-----------------------   G L O B A L   V A R I A B L E S   -----------------------
//-----------------------------------------------------------------------------------
//----------    M O T O R   A C C E L E R A T I O N    ----------
static struct pt motor_protothread;         // Protothread for motor speed updates
const PROGMEM int MOTOR_PTHREAD_DELAY = 20; // Motor speed update delay (in ms)
uint16_t target_motor_pwm[5];               // Target pulse for DC motors
uint16_t current_motor_pwm[5];              // Target pulse for DC motors

const PROGMEM uint16_t motor_increment = 1;

//----------    O T H E R   V A R I A B L E S    ----------
// ROS node handle (makes everything work)
ros::NodeHandle nh;
// Adafruit PWM Driver (for driving servos and motors)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



//-----------------------------------------------------------------------------------
//--------------------   C O D E   B E G I N S   H E R E   --------------------------
//-----------------------------------------------------------------------------------

/* 
  0 < MIN_REVERSE_SPEED < NEUTRAL_SPEED_PWM < MIN_FORWARD_SPEED < 4096

  Cases:
    if TARGET > CURRENT
      if CURRENT == NEUTRAL_SPEED_PWM then CURRENT = MIN_FORWARD_SPEED
      else increment CURRENT
    else if TARGET < CURRENT
      if CURRENT == NEUTRAL_SPEED_PWM then CURRENT = MIN_REVERSE_SPEED
      else decrement CURRENT
    else return
*/
void pthread_motor_helper(int motor_index, int drive_pwm_pin) {
  if (target_motor_pwm[motor_index] > current_motor_pwm[motor_index]) {
    // The target speed is higher than current speed
    current_motor_pwm[motor_index] += motor_increment;
  } else if (target_motor_pwm[motor_index] < current_motor_pwm[motor_index]) {
    // The target speed is lower than the current speed
    current_motor_pwm[motor_index] -= motor_increment;
  } else {
    // We are at target speed
    return;
  }

  pwm.setPWM(drive_pwm_pin, 0, current_motor_pwm[motor_index]);
}

static int pthread_update_DC_motors(struct pt * pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(true) {
    PT_WAIT_UNTIL(pt, (long)millis() - (long)timestamp > MOTOR_PTHREAD_DELAY);
    timestamp = millis();
    
    pthread_motor_helper(DRIVE_ARRAY_INDEX_R,
                         DRIVE_PWM_PIN_R);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_S_R,
                         DRIVE_PWM_PIN_S_R);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_S_L, 
                         DRIVE_PWM_PIN_S_L);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_F_R, 
                         DRIVE_PWM_PIN_F_R);
    pthread_motor_helper(DRIVE_ARRAY_INDEX_F_L,
                         DRIVE_PWM_PIN_F_L);
  }
  PT_END(pt);
}

/*
  This method acts as a callback for the rostopic listener.
  The values in the array SHOULD BE CORRECT PWM PULSE VALUES
*/
void arduino_cmd_callback(const std_msgs::UInt16MultiArray& cmd_msg) {
  // Local value used to read data from the array
  uint16_t valueReadFromArray;

  //----------  A R M   S E R V O S  ----------
  // Arm base servo
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_ARM_BASE]);
  if (ARM_PWM_MIN <= valueReadFromArray && valueReadFromArray <= ARM_PWM_MAX) {
    pwm.setPWM(ARM_PWM_PIN_BASE, 0, valueReadFromArray);
  }
  // Arm shoulder servo
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_ARM_SHOULDER]);
  if (ARM_PWM_MIN <= valueReadFromArray && valueReadFromArray <= ARM_PWM_MAX) {
    pwm.setPWM(ARM_PWM_PIN_SHOULDER, 0, valueReadFromArray);
  }
  // Arm elbow servo
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_ARM_ELBOW]);
  if (ARM_PWM_MIN <= valueReadFromArray && valueReadFromArray <= ARM_PWM_MAX) {
    pwm.setPWM(ARM_PWM_PIN_ELBOW, 0, valueReadFromArray);
  }
  // Arm wrist servo
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_ARM_WRIST]);
  if (ARM_PWM_MIN <= valueReadFromArray && valueReadFromArray <= ARM_PWM_MAX) {
    pwm.setPWM(ARM_PWM_PIN_WRIST, 0, valueReadFromArray);
  }


  //----------  S T E E R   S E R V O S  ----------
  // Update rear steer servo
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_STEER_R]);
  if (STEER_PWM_MIN <= valueReadFromArray && valueReadFromArray <= STEER_PWM_MAX) {
    pwm.setPWM(STEER_PWM_PIN_R, 0, valueReadFromArray);
  }
  // Update front right steer servo
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_STEER_F_R]);
  if (STEER_PWM_MIN <= valueReadFromArray && valueReadFromArray <= STEER_PWM_MAX) {
    pwm.setPWM(STEER_PWM_PIN_F_R, 0, valueReadFromArray);
  }
  // Update front left steer servo
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_STEER_F_L]);
  if (STEER_PWM_MIN <= valueReadFromArray && valueReadFromArray <= STEER_PWM_MAX) {
    pwm.setPWM(STEER_PWM_PIN_F_L, 0, valueReadFromArray);
  }

  //----------  D R I V E   M O T O R S  ----------

  // Update rear drive motor
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_DRIVE_R]);
  if (ABSOLUTE_MIN_PWM <= valueReadFromArray && valueReadFromArray <= ABSOLUTE_MAX_PWM) {
    target_motor_pwm[DRIVE_ARRAY_INDEX_R]   = valueReadFromArray;
  }
  // Update side right drive motors
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_DRIVE_S_R]);
  if (ABSOLUTE_MIN_PWM <= valueReadFromArray && valueReadFromArray <= ABSOLUTE_MAX_PWM) {
    target_motor_pwm[DRIVE_ARRAY_INDEX_S_R] = valueReadFromArray;
  }
  // Update side left drive motors
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_DRIVE_S_L]);
  if (ABSOLUTE_MIN_PWM <= valueReadFromArray && valueReadFromArray <= ABSOLUTE_MAX_PWM) {
    target_motor_pwm[DRIVE_ARRAY_INDEX_S_L] = valueReadFromArray;
  }
  // Update front right drive motor
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_DRIVE_F_R]);
  if (ABSOLUTE_MIN_PWM <= valueReadFromArray && valueReadFromArray <= ABSOLUTE_MAX_PWM) {
    target_motor_pwm[DRIVE_ARRAY_INDEX_F_R] = valueReadFromArray;
  }
  // Update front right drive motor
  valueReadFromArray = ((uint16_t) cmd_msg.data[MSG_INDEX_DRIVE_F_L]);
  if (ABSOLUTE_MIN_PWM <= valueReadFromArray && valueReadFromArray <= ABSOLUTE_MAX_PWM) {
    target_motor_pwm[DRIVE_ARRAY_INDEX_F_L] = valueReadFromArray;
  }


  //----------  G R I P P E R   ----------
 	// Update gripper rotation
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_GRIPPER_ROTATE]);
  if (GRIPPER_ROTATE_PWM_MIN <= valueReadFromArray && valueReadFromArray <= GRIPPER_ROTATE_PWM_MAX) {
    pwm.setPWM(GRIPPER_PWM_PIN_ROTATE, 0,valueReadFromArray);
  }
  // Update gripper claw
  valueReadFromArray =  ((uint16_t) cmd_msg.data[MSG_INDEX_GRIPPER_CLAW]);
  if (GRIPPER_CLAW_PWM_OPEN <= valueReadFromArray && valueReadFromArray <= GRIPPER_CLAW_PWM_CLOSED) {
    pwm.setPWM(GRIPPER_PWM_PIN_CLAW, 0,valueReadFromArray);
  }

  //----------  MAST STEPPER  ----------
  // TODO
}


/***** Subscribe to the following rostopics:
   + arduino_cmd       - Reads an array used to update servos and Motors*/
ros::Subscriber<std_msgs::UInt16MultiArray> sub_arduino_cmd("arduino_cmd", arduino_cmd_callback);

void setup(){
  nh.initNode();    // Initialize ROS node handle
  nh.subscribe(sub_arduino_cmd); // Subscribe to command topic

  // Initialize I2C PWM board
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);

  // Initialize arm servos (set to home position)
  pwm.setPWM(ARM_PWM_PIN_BASE, 0, ARM_PWM_NEUTRAL);
  pwm.setPWM(ARM_PWM_PIN_SHOULDER, 0, ARM_PWM_NEUTRAL);
  pwm.setPWM(ARM_PWM_PIN_ELBOW, 0, ARM_PWM_NEUTRAL);
  pwm.setPWM(ARM_PWM_PIN_WRIST, 0, ARM_PWM_NEUTRAL);

  // Initialize steer servos (set to neutral)
  pwm.setPWM(STEER_PWM_PIN_R, 0, STEER_PWM_NEUTRAL);
  pwm.setPWM(STEER_PWM_PIN_F_R, 0, STEER_PWM_NEUTRAL);
  pwm.setPWM(STEER_PWM_PIN_F_L, 0, STEER_PWM_NEUTRAL);

  // Initialize gripper servos: 
  //	- 0 degres rotation
  //	- open claw
  pwm.setPWM(GRIPPER_PWM_PIN_CLAW, 0, GRIPPER_CLAW_PWM_OPEN);
  pwm.setPWM(GRIPPER_PWM_PIN_ROTATE, 0, GRIPPER_ROTATE_PWM_NEUTRAL);

  // Initialize array for "target" motor pwm (to "NEUTRAL_SPEED_PWM")
  target_motor_pwm[DRIVE_ARRAY_INDEX_R]   = NEUTRAL_SPEED_PWM;
  target_motor_pwm[DRIVE_ARRAY_INDEX_S_R] = NEUTRAL_SPEED_PWM;
  target_motor_pwm[DRIVE_ARRAY_INDEX_S_L] = NEUTRAL_SPEED_PWM;
  target_motor_pwm[DRIVE_ARRAY_INDEX_F_R] = NEUTRAL_SPEED_PWM;
  target_motor_pwm[DRIVE_ARRAY_INDEX_F_L] = NEUTRAL_SPEED_PWM;

  // Initialize array for "current" motor pwm (to "NEUTRAL_SPEED_PWM")
  current_motor_pwm[DRIVE_ARRAY_INDEX_R]   = NEUTRAL_SPEED_PWM;
  current_motor_pwm[DRIVE_ARRAY_INDEX_S_R] = NEUTRAL_SPEED_PWM;
  current_motor_pwm[DRIVE_ARRAY_INDEX_S_L] = NEUTRAL_SPEED_PWM;
  current_motor_pwm[DRIVE_ARRAY_INDEX_F_R] = NEUTRAL_SPEED_PWM;
  current_motor_pwm[DRIVE_ARRAY_INDEX_F_L] = NEUTRAL_SPEED_PWM;

  // Initialize protothread(s)
  PT_INIT(&motor_protothread);
}

void loop(){
  pthread_update_DC_motors(&motor_protothread);
  nh.spinOnce();
  delay(10);
}
                                                                                      
