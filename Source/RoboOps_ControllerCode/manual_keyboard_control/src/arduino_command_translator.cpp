#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <keyboard/Key.h>
#include <inttypes.h>
#include <sstream>
#include <stdio.h>


/*----------    T E S T I N G   C O M M A N D S    ----------
To test this code without running the "Mission Control" code:
  $ rosrun rosserial_python serial_node.py _port:/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<I2C_INDEX>, <servo_1>, etc.]}' */


/*-----------------------------------------------------------------------------------
//------------------    P I N   R E F E R E N C E   T A B L E    --------------------
//-----------------------------------------------------------------------------------
# Servo and DC Motor reference
+---------+---------+---------+--------------------------+
| \\  //  ‖ IN MSG  | OUT MSG | DESCRIPTION              |
|   ||    ‖  ARRAY  |  ARRAY  |                          |
| //  \\  ‖  INDEX  |  INDEX  |                          |
+++++
|         ‖    0    |    0    | Arm Base                 |
|  Arm    ‖    1    |    1    | Arm Shoulder             |
| Servos  ‖    2    |    2    | Arm Elbow                |
|         ‖    3    |    3    | Arm Wrist                |
+---------+---------+---------+--------------------------+
| Gripper ‖    0    |    4    | Gripper vacuum           |
+---------+---------+---------+--------------------------+
|  Drive  ‖    0    |    5    | Rear Wheel               |
| Servos  ‖    1    |    6    | Front Right Wheel        |
|         ‖    2    |    7    | Front Left Wheel         |
+---------+---------+---------+--------------------------+
|         ‖    0    |    8    | Rear Wheel               |
|  Drive  ‖    1    |    9    | Side Right Wheels (BOTH) |
|   DC    ‖    2    |   10    | Side Left Wheels (BOTH)  |
| Motors  ‖    3    |   11    | Front Right Wheel        |
|         ‖    4    |   12    | Front Left Wheels        |
+---------+---------+---------+--------------------------+
|  Mast   ‖    0    |   13    | Mast Stepper             |
+---------+---------+---------+--------------------------+             */


//-----------------------------------------------------------------------------------
//------------------------------   C O N S T A N T S   ------------------------------
//-----------------------------------------------------------------------------------
//----------   P W M    C O N S T A N T S   ----------
#define PWM_FREQUENCY     50
#define PWM_RESOLUTION    4096
#define ABSOLUTE_MAX_PWM  4093  // avoid extremes
#define ABSOLUTE_MIN_PWM  3     // avoid extremes

//----------   A R M   S E R V O   C O N S T A N T S   ----------
// # Hitec HS-785HB 
#define ARM_PWM_MIN          126  // "-315 degrees", min pulse length count (out of 4096@50Hz)
#define ARM_PWM_MAX          504  // "+315 degrees", max pulse length count (out of 4096@50Hz)
#define ARM_PWM_NEUTRAL      315  // "0 degrees", center pulse length count (out of 4096@50Hz)
#define ARM_PWM_360_DEGREES  216  // "360 degrees", one full rotation

//----------   G R I P P E R   C O N S T A N T S   ----------
#define ARM_GRIPPER_OFF 0;
#define ARM_GRIPPER_ON  1;

//----------   S T E E R I N G   S E R V O   C O N S T A N T S   ----------
#define STEER_PWM_MIN          105
#define STEER_PWM_MAX          495 
#define STEER_PWM_NEUTRAL      295
#define STEER_PWM_360_DEGREES  720
//#define STEER_PWM_360_DEGREES  716 // <--- ACTUAL VALUE ???

//----------   D C   M O T O R   C O N S T A N T S   ----------
// # If the motor pwms are too close to neutral (4096/2 +- ~5%)
#define MIN_FORWARD_SPEED_PWM  2248 // Max DC motor pwm
#define MIN_REVERSE_SPEED_PWM  1848 // Min DC motor pwm
#define SPEED_PWM_NEUTRAL      2048 // Stopped DC motor pwm


//-----------------------------------------------------------------------------------
//--------------   A R R A Y   &   M E S S A G E   C O S N T A N T S   --------------
//-----------------------------------------------------------------------------------
//----------    I N   M E S S A G E   I N D E C E S    ----------
// Arm servos
#define IN_MSG_INDEX_ARM_BASE     0
#define IN_MSG_INDEX_ARM_SHOULDER 1
#define IN_MSG_INDEX_ARM_ELBOW    2
#define IN_MSG_INDEX_ARM_WRIST    3
// Arm gripper
#define IN_MSG_INDEX_GRIPPER      4
// Steering servos
#define IN_MSG_INDEX_STEER_R      0
#define IN_MSG_INDEX_STEER_F_R    1
#define IN_MSG_INDEX_STEER_F_L    2
// Drive motors
#define IN_MSG_INDEX_DRIVE_R      0
#define IN_MSG_INDEX_DRIVE_S_R    1
#define IN_MSG_INDEX_DRIVE_S_L    2
#define IN_MSG_INDEX_DRIVE_F_R    3
#define IN_MSG_INDEX_DRIVE_F_L    4
// Mast Stepper
//#define IN_MSG_INDEX_MAST 0  // MIGHT NOT BE USED...

//----------    O U T   M E S S A G E   I N D E C E S    ----------
#define OUT_MSG_INDEX_ARM_BASE      0
#define OUT_MSG_INDEX_ARM_SHOULDER  1
#define OUT_MSG_INDEX_ARM_ELBOW     2
#define OUT_MSG_INDEX_ARM_WRIST     3
#define OUT_MSG_INDEX_GRIPPER       4
#define OUT_MSG_INDEX_STEER_R       5
#define OUT_MSG_INDEX_STEER_F_R     6
#define OUT_MSG_INDEX_STEER_F_L     7
#define OUT_MSG_INDEX_DRIVE_R       8
#define OUT_MSG_INDEX_DRIVE_S_R     9
#define OUT_MSG_INDEX_DRIVE_S_L    10
#define OUT_MSG_INDEX_DRIVE_F_R    11
#define OUT_MSG_INDEX_DRIVE_F_L    12
//#define OUT_MSG_INDEX_MAST_STEPPER  13  // MIGHT NOT BE USED...


//-----------------------------------------------------------------------------------
//----------   R O S   S U S C R I B E R S   A N D   P U B L I S H E R S   ----------
//-----------------------------------------------------------------------------------
// ROS Subscribers
ros::Subscriber * sub_arm_cmd_manual;
ros::Subscriber * sub_steer_cmd_manual;
ros::Subscriber * sub_drive_cmd_manual;

// ROS publisher
ros::Publisher * pub_arduino_cmd;


//-----------------------------------------------------------------------------------
//-----------   M O T O R   A N D   S E R V O   V A R I A B L E S   -----------------
//-----------------------------------------------------------------------------------
// Command publisher array
std_msgs::Int16MultiArray command_message_array;
bool UPDATE_NEEDED = false;


//-----------------------------------------------------------------------------------
//--------------------   C O D E   B E G I N S   H E R E   --------------------------
//-----------------------------------------------------------------------------------

//----------  T R A N S L A T O R S  ---------

/***** Converting angles to PWM pulses
  These functions convert a servo angles (in degrees) into a PWM pulses
  Both arm_angle_to_pulse() and steer_angle_to_pulse() simply call angle_to_pulse()
    with the appropriate values

  Here's some math:
    PWM_pulse = <neurtal_pwm> + (angle/360) * <full_turn_pwm>     
*/
uint16_t angle_to_pulse(int int_angle, int servo_neutral, int servo_full_turn, int servo_min, int servo_max) {
  // Typecast angle to double to prevent overflow
  double angle = int_angle * 1.0;
  uint16_t pulse = (uint16_t) (servo_neutral + (((long)angle*(long)servo_full_turn)/360));
  // Check if within range
  if (pulse < servo_min) { 
    pulse = (uint16_t) servo_min; 
  } else if (pulse > servo_max) {
    pulse = (uint16_t) servo_max;
  }
  return pulse;
}
uint16_t arm_angle_to_pulse(int int_angle) {
  return angle_to_pulse(int_angle, ARM_PWM_NEUTRAL, 
    ARM_PWM_360_DEGREES, ARM_PWM_MIN, ARM_PWM_MAX);
}
uint16_t steer_angle_to_pulse(int int_angle) {
  return angle_to_pulse(int_angle, SPEED_PWM_NEUTRAL, 
    STEER_PWM_360_DEGREES, STEER_PWM_MIN, STEER_PWM_MAX);
}

/***** drive_speed_to_pulse() ###
  Converts a drive speed to a PWM pulse,
    checking if this pulse is within range
    and is not too close to "SPEED_PWM_NEUTRAL"

  !!! THIS FUNCTION SHOULD NEVER RETURN ANY VALUE 
  !!! WITHIN THE MIN FORWARD/BACKWARD PWMs
  !!! OTHER THAN "neutral" (ie SPEED_PWM_NEUTRAL )
*/
uint16_t drive_speed_to_pulse(int int_speed) {
  // Typecast angle to double to prevent overflow
  double speed = int_speed * 1.0 + SPEED_PWM_NEUTRAL ;
  uint16_t pulse = (uint16_t) speed;
  if (pulse > ABSOLUTE_MAX_PWM) { 
    pulse = ABSOLUTE_MAX_PWM;
  } else if (ABSOLUTE_MIN_PWM < 0) {
    pulse = ABSOLUTE_MIN_PWM;
  } else if (MIN_FORWARD_SPEED_PWM > pulse && pulse > MIN_REVERSE_SPEED_PWM) {
    // Too close to "SPEED_PWM_NEUTRAL"... set to "SPEED_PWM_NEUTRAL"
    pulse = SPEED_PWM_NEUTRAL;
  }
  return pulse;
}


//----------  S U B S C R I B E R S / P U B L I S H E R S  ---------

void arm_cmd_manual_callback(const std_msgs::Int16MultiArray& cmd_msg) {
	int16_t  newAngle;	// Reading from array
	uint16_t newPulse;		// Pulse to send to arduino

	// Set arm base
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_ARM_BASE];
	newPulse = arm_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_ARM_BASE] = newPulse;
	// Set arm shoulder
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_ARM_SHOULDER];
	newPulse = arm_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_ARM_SHOULDER] = newPulse;
	// Set arm elbow
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_ARM_ELBOW];
	newPulse = arm_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_ARM_ELBOW] = newPulse;
	// Set arm wrist
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_ARM_WRIST];
	newPulse = arm_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_ARM_WRIST] = newPulse;

	// Gripper
	command_message_array.data[OUT_MSG_INDEX_GRIPPER] = cmd_msg.data[IN_MSG_INDEX_GRIPPER];

	// Signal updated comamnds
	UPDATE_NEEDED = true;
} 

void steer_cmd_manual_callback(const std_msgs::Int16MultiArray& cmd_msg) {
	int16_t  newAngle;	// Reading from array
	uint16_t newPulse;		// Pulse to send to arduino

	// Set steer rear
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_STEER_R];
	newPulse = steer_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_STEER_R] = newPulse;
	// Set steer front right
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_STEER_F_R];
	newPulse = steer_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_STEER_F_R] = newPulse;
	// Set steer front left
	newAngle = (int16_t) cmd_msg.data[IN_MSG_INDEX_STEER_F_L];
	newPulse = steer_angle_to_pulse(newAngle);
	command_message_array.data[OUT_MSG_INDEX_STEER_F_L] = newPulse;

	// Signal updated comamnds
	UPDATE_NEEDED = true;
} 

void drive_cmd_manual_callback(const std_msgs::Int16MultiArray& cmd_msg) {
	int16_t  newSpeed;	// Reading from array
	uint16_t newPulse;	// Pulse to send to arduino

	// Set steer front left
	newSpeed = (int16_t) cmd_msg.data[IN_MSG_INDEX_DRIVE_R];
	newPulse = drive_speed_to_pulse(newSpeed);
	command_message_array.data[OUT_MSG_INDEX_DRIVE_R] = newPulse;
	// Set steer front left
	newSpeed = (int16_t) cmd_msg.data[IN_MSG_INDEX_DRIVE_S_R];
	newPulse = drive_speed_to_pulse(newSpeed);
	command_message_array.data[OUT_MSG_INDEX_DRIVE_S_R] = newPulse;
	// Set steer front left
	newSpeed = (int16_t) cmd_msg.data[IN_MSG_INDEX_DRIVE_S_L];
	newPulse = drive_speed_to_pulse(newSpeed);
	command_message_array.data[OUT_MSG_INDEX_DRIVE_S_L] = newPulse;
	// Set steer front left
	newSpeed = (int16_t) cmd_msg.data[IN_MSG_INDEX_DRIVE_F_R];
	newPulse = drive_speed_to_pulse(newSpeed);
	command_message_array.data[OUT_MSG_INDEX_DRIVE_F_R] = newPulse;
	// Set steer front left
	newSpeed = (int16_t) cmd_msg.data[IN_MSG_INDEX_DRIVE_F_L];
	newPulse = drive_speed_to_pulse(newSpeed);
	command_message_array.data[OUT_MSG_INDEX_DRIVE_F_L] = newPulse;

	// Signal updated comamnds
	UPDATE_NEEDED = true;
} 

//----------  I N I T I A L I Z E R   F U N C T I O N S  ---------
void initialize_command_message_array() {
	// Clear and reinitialize the command array
	command_message_array.data.clear();
	for (int i = 0; i < 14; i ++) {
		command_message_array.data.push_back(0);
	}
	// Set default values
	command_message_array.data[OUT_MSG_INDEX_ARM_BASE]     = ARM_PWM_NEUTRAL;	// Initialize arm servos
	command_message_array.data[OUT_MSG_INDEX_ARM_SHOULDER] = ARM_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_ARM_ELBOW]    = ARM_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_ARM_WRIST]    = ARM_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_GRIPPER]      = ARM_GRIPPER_OFF;	// Initialize gripper
	command_message_array.data[OUT_MSG_INDEX_STEER_R]      = STEER_PWM_NEUTRAL;	// Initialize steering servos
	command_message_array.data[OUT_MSG_INDEX_STEER_F_R]    = STEER_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_STEER_F_L]    = STEER_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_DRIVE_R]      = SPEED_PWM_NEUTRAL;	// Initialize drive motors
	command_message_array.data[OUT_MSG_INDEX_DRIVE_S_R]    = SPEED_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_DRIVE_S_L]    = SPEED_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_DRIVE_F_R]    = SPEED_PWM_NEUTRAL;
	command_message_array.data[OUT_MSG_INDEX_DRIVE_F_L]    = SPEED_PWM_NEUTRAL;
}

int main(int argc, char **argv) {
	// Initialize ROS elements
    ros::init(argc, argv, "arduino_command_translator");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);


    // Create and initialize rostopic subscribers
    sub_arm_cmd_manual = new ros::Subscriber();
    sub_steer_cmd_manual = new ros::Subscriber();
    sub_drive_cmd_manual = new ros::Subscriber();
    *sub_arm_cmd_manual = n.subscribe("arm_cmd_manual", 1000, arm_cmd_manual_callback);
    *sub_steer_cmd_manual = n.subscribe("steer_cmd_manual", 1000, steer_cmd_manual_callback);
    *sub_drive_cmd_manual = n.subscribe("drive_cmd_manual", 1000, drive_cmd_manual_callback);

    // Create and initialize  publisher
    pub_arduino_cmd = new ros::Publisher();
    *pub_arduino_cmd = n.advertise<std_msgs::Int16MultiArray>("arduino_cmd", 1000);

    // Initialize command publisher array
    initialize_command_message_array();

    std::cout << "STARTED PWM TRANSLATOR!!!" << std::endl;

    while (ros::ok()) {

    	// Check fo update to servos/motors
		if (UPDATE_NEEDED) {
			pub_arduino_cmd->publish(command_message_array);
		}

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}