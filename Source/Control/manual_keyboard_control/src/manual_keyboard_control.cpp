#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <keyboard/Key.h>
#include <inttypes.h>
#include <sstream>
#include <stdio.h>

/*
Commands:
  $ roscore
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<servo_1>, <servo_1>, etc.]}'
*/

/* Servo and DC Motor reference
+---------+-------+---------------------------+
| `-..-`  ‖  MSG  | DESCRIPTION               |
| .-``-.  ‖ INDEX |                           |
+=========+=======+===========================+
|         ‖   0   | Arm Base                  |
|  Arm    ‖   1   | Arm Shoulder              |
| Servos  ‖   2   | Arm Elbow                 |
|         ‖   3   | Arm Wrist                 |
+---------+-------+---------------------------+
| Gripper ‖   4   | Gripper Rotate            |
| Gripper ‖   5   | Gripper Claw              |
+---------+-------+---------------------------+
|  Drive  ‖   0   | Back Wheel                |
| Servos  ‖   1   | Front Right Wheel         |
|         ‖   2   | Front Left Wheel          |
+---------+-------+---------------------------+
|  Mast   ‖   0   | Mast Servo                |
+---------+-------+---------------------------+
|         ‖   0   | Rear Wheel                |
|  Drive  ‖   1   | Side Right Wheels (BOTH)  |
|   DC    ‖   2   | Side Left Wheels (BOTH)   |
| Motors  ‖   3   | Front Right Wheel         |
|         ‖   4   | Front Left Wheels         |
+---------+-------+---------------------------+   */

// Camera mast
#define MAST               0
// Arm servos
#define ARM_BASE           0
#define ARM_SHOULDER       1
#define ARM_ELBOW          2
#define ARM_WRIST          3
// Arm gripper
#define ARM_GRIPPER_ROTATE 4
#define ARM_GRIPPER_CLAW   5
// Steering servos
#define STEER_BACK         0
#define STEER_FRONT_RIGHT  1
#define STEER_FRONT_LEFT   2
// Drive motors
#define DRIVE_REAR         0
#define DRIVE_SIDE_RIGHT   1
#define DRIVE_SIDE_LEFT    2
#define DRIVE_FRONT_RIGHT  3
#define DRIVE_FRONT_LEFT   4

// *************************************** LIMIT VALUES *************************************** //
#define GRIPPER_ROTATE_INCREMENT 2
#define GRIPPER_ROTATE_MAX  90
#define GRIPPER_ROTATE_MIN -90
#define GRIPPER_CLAW_INCREMENT   2
#define GRIPPER_CLAW_MAX   100
#define GRIPPER_CLAW_MIN     0

// *************************************** LIMIT VALUES *************************************** //


// ROS variables
ros::Publisher * arm_cmd_manual;
ros::Publisher * steer_cmd_manual;
ros::Publisher * drive_cmd_manual;
ros::Publisher * mast_cmd_manual;


// Keypresses
std::map<uint16_t, bool> keys;
bool CURRENT_VACUUM_STATE = false;

// Arm servo array
int16_t arm_servo[6];
std_msgs::Int16MultiArray arm_servo_message;
bool arm_update_needed = false;

// Steer servo array
int16_t steer_servo[3];
std_msgs::Int16MultiArray steer_servo_message;
bool steer_update_needed = false;

// Drive motor array
int16_t drive_motors[5];
std_msgs::Int16MultiArray drive_motor_message;
bool drive_update_needed = false;

// Mast servo variable
int16_t mast_servo = 0;
std_msgs::Int16 mast_servo_message;
int mast_release_timeout = 0;
bool mast_update_needed = false;


// ************************************************* KEYBOARD HANDLERS ************************************************* //
/***** keyDown() ***
    Changes state of a keyboard key in "keys" map to "true".
    @INPUT keyboard::Key& - the key that was pressed    */
void keyDown(const keyboard::Key& key) {
    keys[key.code] = true;
}

/***** keyUp() ***
    Changes state of a keyboard key in "keys" map to "false".
    @INPUT keyboard::Key& - the key that was released    */
void keyUp(const keyboard::Key& key) {
    keys[key.code] = false;
}
// ************************************************* KEYBOARD HANDLERS ************************************************* //


/***** publish_arm_servo_update() ***
    Publishes a new angle for the specified arm servo.
    @INPUT int servo_index - index of servo to be published     */
void publish_arm_servo_update() {
    arm_servo_message.data[0] = arm_servo[0];
    arm_servo_message.data[1] = arm_servo[1];
    arm_servo_message.data[2] = arm_servo[2];
    arm_servo_message.data[3] = arm_servo[3];
    arm_servo_message.data[4] = arm_servo[4];
    arm_servo_message.data[5] = arm_servo[5];
    arm_cmd_manual->publish(arm_servo_message);
}

void publish_steer_servo_update() {
    steer_servo_message.data[0] = steer_servo[0];
    steer_servo_message.data[1] = steer_servo[1];
    steer_servo_message.data[2] = steer_servo[2];
    steer_cmd_manual->publish(steer_servo_message);
}

/***** publish_drive_motor_update() ***
    Publishes a new angle for the specified drive servo.
    @INPUT int servo_index - index of servo to be published     */
void publish_drive_motor_update() {
    drive_motor_message.data[DRIVE_REAR] = drive_motors[DRIVE_REAR];
    drive_motor_message.data[DRIVE_SIDE_RIGHT] = drive_motors[DRIVE_SIDE_RIGHT];
    drive_motor_message.data[DRIVE_SIDE_LEFT] = drive_motors[DRIVE_SIDE_LEFT];
    drive_motor_message.data[DRIVE_FRONT_RIGHT] = drive_motors[DRIVE_FRONT_RIGHT];
    drive_motor_message.data[DRIVE_FRONT_LEFT] = drive_motors[DRIVE_FRONT_LEFT];
    drive_cmd_manual->publish(drive_motor_message);
}

void publish_mast_servo_update() {
    mast_servo_message.data = mast_servo;
    mast_cmd_manual->publish(mast_servo);
}

/***** initialize_servos() ***
    Initialize all message and servo arrays */
void initialize_servos() {

    // Initialize arm servo array
    arm_servo[0] = 0;
    arm_servo[1] = 0;
    arm_servo[2] = 0;
    arm_servo[3] = 0;
    arm_servo[4] = 0;
    arm_servo[5] = 0;

	// Initialize arm servo update message array
    arm_servo_message.data.clear();
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);

    // Initialize steer servo array
    steer_servo[0] = 0;
    steer_servo[1] = 0;
    steer_servo[2] = 0;

    // Initialize steer servo update message array
    steer_servo_message.data.clear();
    steer_servo_message.data.push_back(0);
    steer_servo_message.data.push_back(0);
    steer_servo_message.data.push_back(0);

    // Initialize drive motor array
    drive_motors[0] = 0;
    drive_motors[1] = 0;
    drive_motors[2] = 0;
    drive_motors[3] = 0;
    drive_motors[4] = 0;

    // Initialize drive motor update message array
    drive_motor_message.data.clear();
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
}

/***** initialize_key_states() ***
    Initialize default key press states */
void initialize_key_states() {
    // Arm
    keys[keyboard::Key::KEY_n] = false; // Base CCW
    keys[keyboard::Key::KEY_m] = false; // Base CW
    keys[keyboard::Key::KEY_u] = false; // Shoulder back
    keys[keyboard::Key::KEY_j] = false; // Shoulder forward
    keys[keyboard::Key::KEY_i] = false; // Elbow up
    keys[keyboard::Key::KEY_k] = false; // Elbow down
    keys[keyboard::Key::KEY_o] = false; // Wrist forward
    keys[keyboard::Key::KEY_l] = false; // Wrist backward
    keys[keyboard::Key::KEY_p] = false; // Return home 

    // Steering
    keys[keyboard::Key::KEY_a] = false; // Steer CCW
    keys[keyboard::Key::KEY_d] = false; // Steer CW
    keys[keyboard::Key::KEY_f] = false; // Steer reset (straight ahead)

    // Motors
    keys[keyboard::Key::KEY_w] = false; // Drive forward
    keys[keyboard::Key::KEY_s] = false; // Drive backward
    keys[keyboard::Key::KEY_x] = false; // Stop motors

    // Gripper
    keys[keyboard::Key::KEY_UP]    = false;    // Gripper Open
    keys[keyboard::Key::KEY_DOWN]  = false;  // Gripper Close
    keys[keyboard::Key::KEY_RIGHT] = false; // Gripper Rotate CW
    keys[keyboard::Key::KEY_LEFT]  = false;  // Gripper Rotate CCW

    // Mast
    keys[keyboard::Key::KEY_q]  = false;  // Mast Rotate CW
    keys[keyboard::Key::KEY_e]  = false;  // Mast Rotate CCW
}


int main(int argc, char **argv) {
    // Initialize ROS elements
    ros::init(argc, argv, "manual_keyboard_control");
    ros::NodeHandle n;  
    ros::Rate loop_rate(20);

    // Publishers for sending commands to motor controller
	arm_cmd_manual = new ros::Publisher();
    steer_cmd_manual = new ros::Publisher();
	drive_cmd_manual = new ros::Publisher();
    mast_cmd_manual = new ros::Publisher();

    *arm_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("arm_cmd_manual", 1000);
    *steer_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("steer_cmd_manual", 1000);
    *drive_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("drive_cmd_manual", 1000);
    *mast_cmd_manual = n.advertise<std_msgs::Int16>("mast_cmd_manual", 1000);
    
    // Keyboard subscribers
    ros::Subscriber keydown = n.subscribe("keyboard/keydown", 10, keyDown);
    ros::Subscriber keyup = n.subscribe("keyboard/keyup", 10, keyUp);

    // Other Initialization code
    initialize_servos();      
    initialize_key_states();

    std::cout << "STARTED PWM PUBLISHER!!!" << std::endl;

    while (ros::ok())
    {
        // Arm home (p)
        if (keys[keyboard::Key::KEY_p]) {
            int target_angle = 0;
            int angle_delta = 0;

            // For each arm servo
            for (int i = 0; i < 6; i++) {
                // If the shoulder is extended and the wrist is not brought in
                // then bring in the wrist (to reduce strain on shoulder servo)
                //      Which means if the current servo is not the wrist, skip it
                //      unless the wrist is retracted!
                if (arm_servo[ARM_SHOULDER] > 55 && arm_servo[ARM_WRIST] != -40 && i != ARM_WRIST) {
                    continue;
                }

                if (i == ARM_BASE && arm_servo[ARM_BASE] != 0 && arm_servo[ARM_SHOULDER] == 30) {
                    // If current servo is the base, and the base is not at 0 degrees,
                    // and the should is currently at 30 degrees (to avoid hitting things)
                    // then set the base target angle to 0 degrees
                    target_angle = 0;
                } else if (i == ARM_SHOULDER && arm_servo[ARM_BASE] != 0) {
                    // If the current servo is the shoulder, and the base is not 0 degrees,
                    // set the shoulder target angle to 30 degrees.
                    target_angle = 30;
                } else if (i == ARM_ELBOW && (arm_servo[ARM_BASE] != 0 || arm_servo[ARM_SHOULDER] > 20)) {
                    // If the current servo is the elbow, and either the base is not at 0 or the
                    // shoulder is bigger than 20 degrees, set the target angle for the elbow 
                    // to whatever the shoulder is plus 30 degrees (to avoid collisions)
                    target_angle = arm_servo[ARM_SHOULDER] + 30;
                } else if (i == ARM_WRIST && arm_servo[ARM_SHOULDER] > 50) {
                    // If the current servo is the wrist, and the shoulder is at more than
                    // 50 degrees, set the wrist angle to -40
                    target_angle = 0;
                } else if (i == ARM_GRIPPER_ROTATE) {
                    target_angle = 0;
                } else if (i == ARM_GRIPPER_CLAW) {
                    continue;
                } else {
                    // Otherwise set the target angle to "home" (ie 0 degrees)
                    target_angle = 0;
                }

                // Save change in angle
                if (arm_servo[i] < target_angle) {
                    angle_delta = 1;
                } else if (arm_servo[i] > target_angle) {
                    angle_delta = -1;
                } else {
                    angle_delta = 0;
                }

                // Double speed base, wrist gripper rotate or gripper claw
                if (i == ARM_BASE || i == ARM_WRIST || i == ARM_GRIPPER_ROTATE || i == ARM_GRIPPER_CLAW) {
                    if (arm_servo[i] - 1 != 0 && arm_servo[i] + 1 != 0) {
                        angle_delta *= 2;
                    }
                }

                // Publish change in angle
                arm_servo[i] += angle_delta;
            }
            arm_update_needed = true;
        }

        // Arm base (m and n)
        if (keys[keyboard::Key::KEY_n]) {
            arm_servo[ARM_BASE] += 1;
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_m]) {
            arm_servo[ARM_BASE] -= 1;
            arm_update_needed = true;
        }

        // Arm shoulder (j and u)
        if (keys[keyboard::Key::KEY_j]) {
            arm_servo[ARM_SHOULDER] += 1;
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_u]) {
            arm_servo[ARM_SHOULDER] -= 1;
            arm_update_needed = true;
        }

        // Arm elbow (i and k)
        if (keys[keyboard::Key::KEY_i]) {
            arm_servo[ARM_ELBOW] += 1;
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_k]) {
            arm_servo[ARM_ELBOW] -= 1;
            arm_update_needed = true;
        }

        // Arm wrist (o and l)
        if (keys[keyboard::Key::KEY_o]) {
            arm_servo[ARM_WRIST] += 1;
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_l]) {
            arm_servo[ARM_WRIST] -= 1;
            arm_update_needed = true;
        }
        
        // Arm gripper rotate (left arrow and right arrow)
        if (keys[keyboard::Key::KEY_LEFT]) {
            if (arm_servo[ARM_GRIPPER_ROTATE] + GRIPPER_ROTATE_INCREMENT >= GRIPPER_ROTATE_MAX) {
                arm_servo[ARM_GRIPPER_ROTATE] = GRIPPER_ROTATE_MAX;
            } else {
                arm_servo[ARM_GRIPPER_ROTATE] += GRIPPER_ROTATE_INCREMENT;
            }
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_RIGHT]) {
            if (arm_servo[ARM_GRIPPER_ROTATE] - GRIPPER_ROTATE_INCREMENT <= GRIPPER_ROTATE_MIN) {
                arm_servo[ARM_GRIPPER_ROTATE] = GRIPPER_ROTATE_MIN;
            } else {
                arm_servo[ARM_GRIPPER_ROTATE] -= GRIPPER_ROTATE_INCREMENT;
            }
            arm_update_needed = true;
        }

        // Arm gripper rotate (up arrow and down arrow)
        if (keys[keyboard::Key::KEY_UP]) {
            // OPEN GRIPPER
            if (arm_servo[ARM_GRIPPER_CLAW] - GRIPPER_CLAW_INCREMENT <= GRIPPER_CLAW_MIN) {
                arm_servo[ARM_GRIPPER_CLAW] = GRIPPER_CLAW_MIN;
            } else {
                arm_servo[ARM_GRIPPER_CLAW] -= GRIPPER_CLAW_INCREMENT;
            }
            arm_update_needed = true;
        } else if (keys[keyboard::Key::KEY_DOWN]) {
            // CLOSE GRIPPER
            if (arm_servo[ARM_GRIPPER_CLAW] + GRIPPER_CLAW_INCREMENT >= GRIPPER_CLAW_MAX) {
                arm_servo[ARM_GRIPPER_CLAW] = GRIPPER_CLAW_MAX;
            } else {
                arm_servo[ARM_GRIPPER_CLAW] += GRIPPER_CLAW_INCREMENT;
            }
            arm_update_needed = true;
        }

        // Steering (a and d and f)
        // q = CCW; e = cw
        if (keys[keyboard::Key::KEY_a]) {
            steer_servo[STEER_BACK] += 3;
            steer_servo[STEER_FRONT_RIGHT] += 3;
            steer_servo[STEER_FRONT_LEFT] += 3;
            steer_update_needed = true;
        } else if (keys[keyboard::Key::KEY_d]) {
            steer_servo[STEER_BACK] -= 3;
            steer_servo[STEER_FRONT_RIGHT] -= 3;
            steer_servo[STEER_FRONT_LEFT] -= 3;
            steer_update_needed = true;
        }
	    if (keys[keyboard::Key::KEY_f]) {
            steer_servo[STEER_BACK] = 0;
            steer_servo[STEER_FRONT_RIGHT] = 0;
            steer_servo[STEER_FRONT_LEFT] = 0;
            steer_update_needed = true;
        }

        // Drive motors (w and s and x)
        if (keys[keyboard::Key::KEY_w]) {
            (drive_motors[DRIVE_REAR] < 2000) ? drive_motors[DRIVE_REAR] += 100 : drive_motors[DRIVE_REAR] = 2000;
            (drive_motors[DRIVE_SIDE_RIGHT] < 2000) ? drive_motors[DRIVE_SIDE_RIGHT] += 100 : drive_motors[DRIVE_SIDE_RIGHT] = 2000;
            (drive_motors[DRIVE_SIDE_LEFT] < 2000) ? drive_motors[DRIVE_SIDE_LEFT] += 100 : drive_motors[DRIVE_SIDE_LEFT] = 2000;
            (drive_motors[DRIVE_FRONT_RIGHT] < 2000) ? drive_motors[DRIVE_FRONT_RIGHT] += 100 : drive_motors[DRIVE_FRONT_RIGHT] = 2000;
            (drive_motors[DRIVE_FRONT_LEFT] < 2000) ? drive_motors[DRIVE_FRONT_LEFT] += 100 : drive_motors[DRIVE_FRONT_LEFT] = 2000;
            drive_update_needed = true;
        } else if (keys[keyboard::Key::KEY_s]) {
            (drive_motors[DRIVE_REAR] > -2000) ? drive_motors[DRIVE_REAR] -= 100 : drive_motors[DRIVE_REAR] = -2000;
            (drive_motors[DRIVE_SIDE_RIGHT] > -2000) ? drive_motors[DRIVE_SIDE_RIGHT] -= 100 : drive_motors[DRIVE_SIDE_RIGHT] = -2000;
            (drive_motors[DRIVE_SIDE_LEFT] > -2000) ? drive_motors[DRIVE_SIDE_LEFT] -= 100 : drive_motors[DRIVE_SIDE_LEFT] = -2000;
            (drive_motors[DRIVE_FRONT_LEFT] > -2000) ? drive_motors[DRIVE_FRONT_RIGHT] -= 100 : drive_motors[DRIVE_FRONT_RIGHT] = -2000;
            (drive_motors[DRIVE_FRONT_LEFT] > -2000) ? drive_motors[DRIVE_FRONT_LEFT] -= 100 : drive_motors[DRIVE_FRONT_LEFT] = -2000;
            drive_update_needed = true;
        }
		if (keys[keyboard::Key::KEY_x]) {
            drive_motors[DRIVE_REAR] = 0;
            drive_motors[DRIVE_SIDE_RIGHT] = 0;
            drive_motors[DRIVE_SIDE_LEFT] = 0;
            drive_motors[DRIVE_FRONT_RIGHT] = 0;
            drive_motors[DRIVE_FRONT_LEFT] = 0;
            drive_update_needed = true;
        }

        // Mast Servo (q & w)
        if (keys[keyboard::Key::KEY_q]) {
            mast_servo = 5;
            if (0 < mast_release_timeout) {
                // If the mast was not previously moving
                mast_release_timeout = 0;
                mast_update_needed = true;
            }
        } else if (keys[keyboard::Key::KEY_e]) {
            mast_servo = -5;
            if (0 < mast_release_timeout) {
                // If the mast was not previously moving
                mast_release_timeout = 0;
                mast_update_needed = true;
            }
        } else if (mast_release_timeout < 5) {
            // Only execute this code if mast was previously moving
            mast_release_timeout++;
            mast_servo = 0;
            mast_update_needed = true;
        }

        // Check if comamnd updates are needed
        if (arm_update_needed) {
            arm_update_needed = false;
            publish_arm_servo_update();
        }
        if (steer_update_needed) {
            steer_update_needed = false;
            publish_steer_servo_update();
        }
        if (drive_update_needed) {
            drive_update_needed = false;
            publish_drive_motor_update();
        }
        if (mast_update_needed) {
            mast_update_needed = false;
            publish_mast_servo_update();
        }

        // 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
