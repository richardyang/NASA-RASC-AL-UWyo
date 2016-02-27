#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <inttypes.h>
#include <sstream>
#include <stdio.h>
#include <termios.h> //termios, TCSANOW, ECHO, ICANON
#include <unistd.h> //STDIN_FILENO

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

int main(int argc, char **argv)
{


    ros::init(argc, argv, "Richards_Dick");
    ros::NodeHandle n;
    ros::Publisher cmd_drive_steer = n.advertise<std_msgs::UInt16MultiArray>("drive_steer", 1000);
    ros::Publisher cmd_drive_speed = n.advertise<std_msgs::Int16>("drive_speed", 1000);
     ros::Publisher cmd_arm = n.advertise<std_msgs::UInt16MultiArray>("arm", 1000);
    //ros::Publisher servo_cb_R = n.advertise<std_msgs::UInt16>("servo_R", 1000);
    //ros::Publisher servo_cb_L = n.advertise<std_msgs::UInt16>("servo_L", 1000);
    ros::Rate loop_rate(15);
    //std_msgs::UInt16 right, left;
    std::cout << "HERE!!!" << std::endl;
    std_msgs::UInt16MultiArray drive_servos;
    std_msgs::UInt16MultiArray arm_servos;
    drive_servos.data.clear();
    arm_servos.data.clear();
    drive_servos.data.push_back(90);
    drive_servos.data.push_back(90);
    std_msgs::Int16 motor_speed;
    motor_speed.data = 0;
    //left.data=10;

    while (ros::ok())
    {
        int c = getch();
        switch(c) {
            // DRIVE SERVOS
            case 'a':
             std::cout << "a pressed" << std::endl;
             drive_servos.data[0]=(drive_servos.data[0]<=15)?(15):(drive_servos.data[0]-5);
             drive_servos.data[1]=(drive_servos.data[1]>=165)?(165):(drive_servos.data[1]+5);
            break;
            case 'd':
             std::cout << "d pressed" << std::endl;
             drive_servos.data[0]=(drive_servos.data[0]>=165)?(165):(drive_servos.data[0]+5);
             drive_servos.data[1]=(drive_servos.data[1]<=15)?(15):(drive_servos.data[1]-5);
            break;
            case 'c':
             std::cout << "c pressed" << std::endl;
             drive_servos.data[0]=(90);
             drive_servos.data[1]=(90);
            break;
            
            // MOTORS
            case 'w':
             std::cout << "w pressed" << std::endl;
             motor_speed.data=(motor_speed.data>=250)?(250):(motor_speed.data+50);
            break;
            case 's':
             std::cout << "s pressed" << std::endl;
             motor_speed.data=(motor_speed.data<=-250)?(-250):(motor_speed.data-50);
            break;
            case 'x':
             std::cout << "x pressed" << std::endl;
             motor_speed.data=(0);
            break;
            
            
            // Moves the BASE of the arm to the (1)right or the (3)left
            case '1':
            std::cout << '1 pressed base servo goes left' << std::endl;
            arm_servos.data[0]=(arm_servos.data[0]<=15)?(15):(arm_servos.data[0]-5);
            break;
            case '3':
            std::cout << '3 pressed base servo goes right' << std::endl;
            arm_servos.data[0]=(arm_servos.data[0]>165)?(165):(arm_servos.data[0]+5);
            break;
           
            // Moves the SHOULDER of the arm (7)up or (4)down
            case '7':
            std::cout << '7 pressed shoulder servo goes up' << std::endl;
            arm_servos.data[1]=(arm_servos.data[1]>165)?(165):(arm_servos.data[1]+5);
            break;
            case '4':
            std::cout << '4 pressed shoulder servo goes down' << std::endl;
            arm_servos.data[1]=(arm_servos.data[1]<=15)?(15):(arm_servos.data[1]-5);
            break;
            
            // Moves the ELBOW of the arm (8)up or (5)down
            case '8':
            std::cout << '8 pressed elbow servo goes up' << std::endl;
            arm_servos.data[2]=(arm_servos.data[2]>165)?(165):(arm_servos.data[2]+5);
            break;
            case '5':
            std::cout << '5 pressed elbow servo goes down' << std::endl;
            arm_servos.data[2]=(arm_servos.data[2]<=15)?(15):(arm_servos.data[2]-5);
            break;
            
            // Moves the WRIST of the arm (9)up or (6)down
            case '9':
            std::cout << '9 pressed wrist servo goes up' << std::endl;
            arm_servos.data[3]=(arm_servos.data[3]>165)?(165):(arm_servos.data[3]+5);
            break;
            case '6':
            std::cout << '6 pressed wrist servo  goes down'' << std::endl;
            arm_servos.data[3]=(arm_servos.data[2]<=15)?(15):(arm_servos.data[2]-5);
            break;
            
            //(0)Ready position that moves the SHOULDER (60 degrees) nad the ELBOW (90 degrees) of the arm at the same time
            case '0':
            std::cout << '0 pressed ready position' << std::endl;
            arm_servos.data[1]=60;
            arm_servos.data[2]=80;
            break;
        }


        std::cout << "Front servo angle: " << drive_servos.data[1] << std::endl;
        std::cout << "Back servo angle: " << drive_servos.data[0] << std::endl;
        
        std::cout << "Base of arm servo angle: " << arm_servos.data[0] << std::endl;
        std::cout << "Shoulder of arm servo angle: " << arm_servos.data[1] << std::endl;
        std::cout << "Elbow of arm servo angle: " << arm_servos.data[2] << std::endl;
        std::cout << "Wrist of arm servo angle: " << arm_servos.data[3] << std::endl;
        
        std::cout << "Motor speed: " << motor_speed.data << std::endl;
        cmd_drive_steer.publish(drive_servos);
        cmd_drive_speed.publish(motor_speed);
        cmd_arm.publish(arm_servos);
        //servo_cb_R.publish(right);
        //servo_cb_L.publish(left);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
