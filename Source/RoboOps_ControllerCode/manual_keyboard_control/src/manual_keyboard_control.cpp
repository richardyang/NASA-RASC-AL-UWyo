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
+--------+-----+-------+-------+---------------------------+
| `-..-` ‖ I2C |  MSG  | ARRAY | DESCRIPTION               |
| .-``-. ‖ PIN | INDEX | INDEX |                           |
+========+=====+=======+=======+===========================+
|        ‖  0  |   0   |   0   | Arm Base                  |
|  Arm   ‖  1  |   1   |   1   | Arm Shoulder              |
| Servos ‖  2  |   2   |   2   | Arm Elbow                 |
|        ‖  3  |   3   |   3   | Arm Wrist                 |
+--------+-----+-------+-------+---------------------------+
|  Drive ‖  4  |   0   |   0   | Back Wheel                |
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
// Camera mast
#define MAST 0
// Arm servos
#define ARM_BASE 0
#define ARM_SHOULDER 1
#define ARM_ELBOW 2
#define ARM_WRIST 3
// Steering servos
#define STEER_BACK 0
#define STEER_FRONT_RIGHT 1
#define STEER_FRONT_LEFT 2
// Drive motors
#define DRIVE_REAR 0
#define DRIVE_SIDE_RIGHT 1
#define DRIVE_SIDE_LEFT 2
#define DRIVE_FRONT_RIGHT 3
#define DRIVE_FRONT_LEFT 4


// ROS variables
ros::Publisher * arm_cmd_manual;
ros::Publisher * steer_cmd_manual;
ros::Publisher * drive_cmd_manual;


// Keypresses
std::map<uint16_t, bool> keys;
bool CURRENT_VACUUM_STATE = false;

// Arm servo array
int16_t arm_servo[4];
std_msgs::Int16MultiArray arm_servo_message;

// Steer servo array
int16_t steer_servo[3];

// Drive motor array
int16_t drive_motors[8];
std_msgs::Int16MultiArray drive_motor_message;


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
void publish_arm_servo_update(int servo_index) {
    arm_servo_message.data[0] = servo_index;
    arm_servo_message.data[1] = arm_servo[servo_index];
    arm_cmd_manual->publish(arm_servo_message);
}

/***** publish_drive_motor_update() ***
    Publishes a new angle for the specified drive servo.
    @INPUT int servo_index - index of servo to be published     */
void publish_drive_motor_update() {
    drive_motor_message.data[0] = drive_motors[0];
    drive_motor_message.data[1] = drive_motors[1];
    drive_motor_message.data[2] = drive_motors[2];
    drive_motor_message.data[3] = drive_motors[3];
    drive_motor_message.data[4] = drive_motors[4];
    drive_motor_message.data[5] = steer_servo[0];
    drive_motor_message.data[6] = steer_servo[1];
    drive_motor_message.data[7] = steer_servo[2];
    drive_cmd_manual->publish(drive_motor_message);
}

/***** initialize_servos() ***
    Initialize all message and servo arrays */
void initialize_servos() {

	// Initialize arm servo update message array
    arm_servo_message.data.clear();
    arm_servo_message.data.push_back(0);
    arm_servo_message.data.push_back(0);

    // Initialize arm servo array
    arm_servo[0] = 0;
    arm_servo[1] = 0;
    arm_servo[2] = 0;
    arm_servo[3] = 0;

    // Initialize drive motor update message array
    drive_motor_message.data.clear();
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);
    drive_motor_message.data.push_back(0);

    // Initialize steer servo array
    steer_servo[0] = 0;
    steer_servo[1] = 0;
    steer_servo[2] = 0;

    // Initialize drive motor array
    drive_motors[0] = 0;
    drive_motors[1] = 0;
    drive_motors[2] = 0;
    drive_motors[3] = 0;
    drive_motors[4] = 0;

    // Put all arm servos at 0 degrees
    for (int i = 0; i < 4; i++) {
        publish_arm_servo_update(i);
    }


		// Set all drive motor speeds to 0
    publish_drive_motor_update();

}

/***** initialize_key_states() ***
    Initialize default key press states */
void initialize_key_states() {
    keys[keyboard::Key::KEY_n] = false; // Base CCW
    keys[keyboard::Key::KEY_m] = false; // Base CW
    keys[keyboard::Key::KEY_u] = false; // Shoulder back
    keys[keyboard::Key::KEY_j] = false; // Shoulder forward
    keys[keyboard::Key::KEY_i] = false; // Elbow up
    keys[keyboard::Key::KEY_k] = false; // Elbow down
    keys[keyboard::Key::KEY_o] = false; // Wrist forward
    keys[keyboard::Key::KEY_l] = false; // Wrist backward
    keys[keyboard::Key::KEY_p] = false; // Return home 
    keys[keyboard::Key::KEY_b] = false; // Toggle vacuum

    keys[keyboard::Key::KEY_q] = false; // Steer CCW
    keys[keyboard::Key::KEY_e] = false; // Steer CW
    keys[keyboard::Key::KEY_r] = false; // Steer back to straight

    keys[keyboard::Key::KEY_w] = false; // Drive forward
    keys[keyboard::Key::KEY_s] = false; // Drive backward
    keys[keyboard::Key::KEY_x] = false; // Stop motors

}


int main(int argc, char **argv) {
    // Initialize ROS elements
    ros::init(argc, argv, "continuous_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    // Publishers to motor controller
		arm_cmd_manual = new ros::Publisher();
		drive_cmd_manual = new ros::Publisher();

    *arm_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("arm_cmd_manual", 1000);
    *drive_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("drive_cmd_manual", 1000);
    
    // Keyboard subscribers
    ros::Subscriber keydown = n.subscribe("keyboard/keydown", 10, keyDown);
    ros::Subscriber keyup = n.subscribe("keyboard/keyup", 10, keyUp);

    std::cout << "STARTED PWM PUBLISHER!!!" << std::endl;

    initialize_servos();      
    initialize_key_states();

    while (ros::ok())
    {
        // Arm home (p)
        if (keys[keyboard::Key::KEY_p]) {
            int target_angle = 0;
            int angle_delta = 0;

            for (int i = 0; i < 4; i++) {
                if (arm_servo[1] > 55 && arm_servo[3] != -40 && i != 3) {
                    continue;
                }

                if (i == 0 && arm_servo[0] != 0 && arm_servo[1] != 30) {
                    target_angle = arm_servo[0];
                } else if (i == 1 && arm_servo[0] != 0) {
                    target_angle = 30;
                } else if (i == 2 && (arm_servo[0] != 0 || arm_servo[1] > 20)) {
                    target_angle = arm_servo[1] + 30;
                } else if (i == 3 && arm_servo[1] > 50) {
                    target_angle = -40;
                } else {
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

                // Double speed if not the base
                if (i == 0 || i == 3) {
                    if (arm_servo[i] - 1 != 0 && arm_servo[i] + 1 != 0) {
                        angle_delta *= 2;
                    }
                }

                // Publish change in angle
                arm_servo[i] += angle_delta;
                publish_arm_servo_update(i);
            }
        }

        // Arm base (m and n)
        if (keys[keyboard::Key::KEY_n]) {
            arm_servo[ARM_BASE] += 1;
            publish_arm_servo_update(ARM_BASE);
        } else if (keys[keyboard::Key::KEY_m]) {
            arm_servo[ARM_BASE] -= 1;
            publish_arm_servo_update(ARM_BASE);
        }

        // Arm shoulder (j and u)
        if (keys[keyboard::Key::KEY_j]) {
            arm_servo[ARM_SHOULDER] += 1;
            publish_arm_servo_update(ARM_SHOULDER);
        } else if (keys[keyboard::Key::KEY_u]) {
            arm_servo[ARM_SHOULDER] -= 1;
            publish_arm_servo_update(ARM_SHOULDER);
        }

        // Arm elbow (i and k)
        if (keys[keyboard::Key::KEY_i]) {
            arm_servo[ARM_ELBOW] += 1;
            publish_arm_servo_update(ARM_ELBOW);
        } else if (keys[keyboard::Key::KEY_k]) {
            arm_servo[ARM_ELBOW] -= 1;
            publish_arm_servo_update(ARM_ELBOW);
        }

        // Arm wrist (o and l)
        if (keys[keyboard::Key::KEY_o]) {
            arm_servo[ARM_WRIST] += 1;
            publish_arm_servo_update(ARM_WRIST);
        } else if (keys[keyboard::Key::KEY_l]) {
            arm_servo[ARM_WRIST] -= 1;
            publish_arm_servo_update(ARM_WRIST);
        }
        
        // Arm gripper (b)
        if (keys[keyboard::Key::KEY_b]) {
            CURRENT_VACUUM_STATE = !CURRENT_VACUUM_STATE;
            keys[keyboard::Key::KEY_b] = false;
            arm_servo_message.data[0] = 4;  
            CURRENT_VACUUM_STATE ? arm_servo_message.data[1] = 1 : arm_servo_message.data[1] = 0;
            arm_cmd_manual->publish(arm_servo_message);
        }

        // Steering (a and d and f)
        // q = CCW; e = cw
        if (keys[keyboard::Key::KEY_a]) {
            steer_servo[STEER_BACK] += 3;
            steer_servo[STEER_FRONT_RIGHT] += 3;
            steer_servo[STEER_FRONT_LEFT] += 3;
            publish_drive_motor_update();
        } else if (keys[keyboard::Key::KEY_d]) {
            steer_servo[STEER_BACK] -= 3;
            steer_servo[STEER_FRONT_RIGHT] -= 3;
            steer_servo[STEER_FRONT_LEFT] -= 3;
            publish_drive_motor_update();
        }
	    if (keys[keyboard::Key::KEY_f]) {
            steer_servo[STEER_BACK] = 0;
            steer_servo[STEER_FRONT_RIGHT] = 0;
            steer_servo[STEER_FRONT_LEFT] = 0;
            publish_drive_motor_update();
        }

        // Drive motors (w and s and x)
        if (keys[keyboard::Key::KEY_w]) {

            (drive_motors[DRIVE_REAR] < 2000) ? drive_motors[DRIVE_REAR] += 100 : drive_motors[DRIVE_REAR] = 2000;
            (drive_motors[DRIVE_SIDE_RIGHT] < 2000) ? drive_motors[DRIVE_SIDE_RIGHT] += 100 : drive_motors[DRIVE_SIDE_RIGHT] = 2000;
            (drive_motors[DRIVE_SIDE_LEFT] < 2000) ? drive_motors[DRIVE_SIDE_LEFT] += 100 : drive_motors[DRIVE_SIDE_LEFT] = 2000;
            (drive_motors[STEER_FRONT_RIGHT] < 2000) ? drive_motors[DRIVE_FRONT_RIGHT] += 100 : drive_motors[DRIVE_FRONT_RIGHT] = 2000;
            (drive_motors[DRIVE_FRONT_LEFT] < 2000) ? drive_motors[DRIVE_FRONT_LEFT] += 100 : drive_motors[DRIVE_FRONT_LEFT] = 2000;
            publish_drive_motor_update();
        } else if (keys[keyboard::Key::KEY_s]) {
            (drive_motors[DRIVE_REAR] > -2000) ? drive_motors[DRIVE_REAR] -= 100 : drive_motors[DRIVE_REAR] = -2000;
            (drive_motors[DRIVE_SIDE_RIGHT] > -2000) ? drive_motors[DRIVE_SIDE_RIGHT] -= 100 : drive_motors[DRIVE_SIDE_RIGHT] = -2000;
            (drive_motors[DRIVE_SIDE_LEFT] > -2000) ? drive_motors[DRIVE_SIDE_LEFT] -= 100 : drive_motors[DRIVE_SIDE_LEFT] = -2000;
            (drive_motors[STEER_FRONT_RIGHT] > -2000) ? drive_motors[DRIVE_FRONT_RIGHT] -= 100 : drive_motors[DRIVE_FRONT_RIGHT] = -2000;
            (drive_motors[DRIVE_FRONT_LEFT] > -2000) ? drive_motors[DRIVE_FRONT_LEFT] -= 100 : drive_motors[DRIVE_FRONT_LEFT] = -2000;
            publish_drive_motor_update();
        }
				if (keys[keyboard::Key::KEY_x]) {
            drive_motors[DRIVE_REAR] = 0;
            drive_motors[DRIVE_SIDE_RIGHT] = 0;
            drive_motors[DRIVE_SIDE_LEFT] = 0;
            drive_motors[DRIVE_FRONT_RIGHT] = 0;
            drive_motors[DRIVE_FRONT_LEFT] = 0;
            publish_drive_motor_update();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
