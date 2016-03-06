#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <keyboard/Key.h>
#include <inttypes.h>
#include <sstream>
#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h> //STDIN_FILENO

// Keyboard keys
std::map<uint16_t, bool> keys;

/*
Commands:
  $ roscore
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<servo_1>, <servo_1>, etc.]}'
*/

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON); // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings
    int c = getchar(); // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

void keyDown(const keyboard::Key& key) {
    keys[key.code] = true;
}

void keyUp(const keyboard::Key& key) {
    keys[key.code] = false;
}

int main(int argc, char **argv)
{   ros::init(argc, argv, "continuous_control");
    ros::NodeHandle n;
    ros::Publisher arm_cmd_manual = n.advertise<std_msgs::Int16MultiArray>("arm_cmd_manual", 1000);
    ros::Subscriber keydown = n.subscribe("keyboard/keydown", 10, keyDown);
    ros::Subscriber keyup = n.subscribe("keyboard/keyup", 10, keyUp);
    ros::Rate loop_rate(20);
    std::cout << "STARTED PWM PUBLISHER!!!" << std::endl;

    // Arm servo array
    int16_t arm_servos[4];
    arm_servos[0] = 0;
    arm_servos[1] = 0;
    arm_servos[2] = 0;
    arm_servos[3] = 0;

    // Arm servo update array
    std_msgs::Int16MultiArray arm_servo_update;
    arm_servo_update.data.clear();
    arm_servo_update.data.push_back(0);
    arm_servo_update.data.push_back(0);


    //Put all servos at 0 degrees
    for (int i = 0; i < 4; i++) {
        arm_servo_update.data[0] = (int16_t) i;
        arm_servo_update.data[1] = 0;
        arm_cmd_manual.publish(arm_servo_update);
    }    

    // Array of pressed keys
    keys[keyboard::Key::KEY_n] = false;
    keys[keyboard::Key::KEY_m] = false;

    keys[keyboard::Key::KEY_u] = false;
    keys[keyboard::Key::KEY_j] = false;

    keys[keyboard::Key::KEY_i] = false;
    keys[keyboard::Key::KEY_k] = false;

    keys[keyboard::Key::KEY_o] = false;
    keys[keyboard::Key::KEY_l] = false;

    keys[keyboard::Key::KEY_p] = false;

    while (ros::ok())
    {
        if (keys[keyboard::Key::KEY_p]) {
            int target_angle = 0;
            int angle_delta = 0;

            for (int i = 0; i < 4; i++) {
                if (arm_servos[1] > 55 && arm_servos[3] != -40 && i != 3) {
                    continue;
                }

                if (i == 0 && arm_servos[0] != 0 && arm_servos[1] != 30) {
                    target_angle = arm_servos[0];
                } else if (i == 1 && arm_servos[0] != 0) {
                    target_angle = 30;
                } else if (i == 2 && (arm_servos[0] != 0 || arm_servos[1] > 20)) {
                    target_angle = arm_servos[1] + 30;
                } else if (i == 3 && arm_servos[1] > 50) {
                    target_angle = -40;
                } else {
                    target_angle = 0;
                }

                // Save change in angle
                if (arm_servos[i] < target_angle) {
                    angle_delta = 1;
                } else if (arm_servos[i] > target_angle) {
                    angle_delta = -1;
                } else {
                    angle_delta = 0;
                }

                // Double speed if not the base
                if (i == 0 || i == 3) {
                    if (arm_servos[i] - 1 != 0 && arm_servos[i] + 1 != 0) {
                        angle_delta *= 2;
                    }
                }

                // Publish change in angle
                arm_servos[i] += angle_delta;
                arm_servo_update.data[0] = (int16_t) i;
                arm_servo_update.data[1] = arm_servos[i];
                arm_cmd_manual.publish(arm_servo_update);
            }
        }

        // Arm base (m and n)
        if (keys[keyboard::Key::KEY_n]) {
            arm_servos[0] += 1;
            arm_servo_update.data[0] = 0;
            arm_servo_update.data[1] = arm_servos[0];
            arm_cmd_manual.publish(arm_servo_update);
        } else if (keys[keyboard::Key::KEY_m]) {
            arm_servos[0] -= 1;
            arm_servo_update.data[0] = 0;
            arm_servo_update.data[1] = arm_servos[0];
            arm_cmd_manual.publish(arm_servo_update);
        }

        if (keys[keyboard::Key::KEY_j]) {
            arm_servos[1] += 1;
            arm_servo_update.data[0] = 1;
            arm_servo_update.data[1] = arm_servos[1];
            arm_cmd_manual.publish(arm_servo_update);
        } else if (keys[keyboard::Key::KEY_u]) {
            arm_servos[1] -= 1;
            arm_servo_update.data[0] = 1;
            arm_servo_update.data[1] = arm_servos[1];
            arm_cmd_manual.publish(arm_servo_update);
        }

        if (keys[keyboard::Key::KEY_i]) {
            arm_servos[2] += 1;
            arm_servo_update.data[0] = 2;
            arm_servo_update.data[1] = arm_servos[2];
            arm_cmd_manual.publish(arm_servo_update);
        } else if (keys[keyboard::Key::KEY_k]) {
            arm_servos[2] -= 1;
            arm_servo_update.data[0] = 2;
            arm_servo_update.data[1] = arm_servos[2];
            arm_cmd_manual.publish(arm_servo_update);
        }

        if (keys[keyboard::Key::KEY_o]) {
            arm_servos[3] += 1;
            arm_servo_update.data[0] = 3;
            arm_servo_update.data[1] = arm_servos[3];
            arm_cmd_manual.publish(arm_servo_update);
        } else if (keys[keyboard::Key::KEY_l]) {
            arm_servos[3] -= 1;
            arm_servo_update.data[0] = 3;
            arm_servo_update.data[1] = arm_servos[3];
            arm_cmd_manual.publish(arm_servo_update);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
