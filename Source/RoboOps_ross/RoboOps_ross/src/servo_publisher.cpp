#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/UInt16.h>
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
    ros::Publisher servo_cb = n.advertise<std_msgs::UInt16>("servo", 1000);
    //ros::Publisher servo_cb_R = n.advertise<std_msgs::UInt16>("servo_R", 1000);
    //ros::Publisher servo_cb_L = n.advertise<std_msgs::UInt16>("servo_L", 1000);
    ros::Rate loop_rate(15);
    //std_msgs::UInt16 right, left;
    std_msgs::UInt16 right;
    right.data=10;
    //left.data=10;

    while (ros::ok())
    {
        int c = getch();
        switch(c) {
            case 'w':
             std::cout << 'w pressed' << std::endl;
             right.data=(right.data>=170)?(170):(right.data+5);
            break;
            case 's':
             std::cout << 's pressed' << std::endl;
             right.data=(right.data<=10)?(10):(right.data-5);
            break;
            case 'q':
             std::cout << 'a pressed' << std::endl;
//             left.data=(left.data>=170)?(170):(left.data+5);
            break;
            case 'e':
             std::cout << 'd pressed' << std::endl;
//             left.data=(left.data>=170)?(170):(left.data-5);
            break;
            case 'a':
             std::cout << 'a pressed' << std::endl;
//             left.data=(left.data>=170)?(170):(left.data+5);
            break;
            case 'd':
             std::cout << 'a pressed' << std::endl;
//             left.data=(left.data>=170)?(170):(left.data+5);
            break;
        }

        std::cout << "Current right servo position:" << right.data << std::endl;
        //std::cout << "Current left servo position:" << left.data << std::endl;
        servo_cb.publish(right);
        //servo_cb_R.publish(right);
        //servo_cb_L.publish(left);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
