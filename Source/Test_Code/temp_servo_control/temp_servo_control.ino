#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"

/*
Commands:
  $ roscore
  $ rosrun rosserial_python serial_node.py _port:=/dev/<PORT NUMBER>
  $ rostopic pub arm_cmd std_msgs/UInt16MultiArray '{data: [<servo_1>, <servo_1>, etc.]}'
*/
ros::NodeHandle nh;

/* arm_servos
PIN | INDEX | DESCRIPTION
----+-------+-----------------
  6 |   0   | Arm Base
  9 |   1   | Arm Shoulder
 10 |   2   | Arm Elbow
 11 |   3   | Arm Wrist
*/
Servo arm_servos[4];

/* drive_servos
PIN | INDEX | DESCRIPTION
----+-------+-----------------
  3 |   0   | Back Wheels
  5 |   1   | Front Wheels
*/
Servo drive_servos[2];

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void publish(String msg) {
  char const * temp = msg.c_str();
  str_msg.data = temp;
  chatter.publish(&str_msg);
}

void update_arm_servos(const std_msgs::UInt16MultiArray&  cmd_msg){
  //arm_servos[0].write(cmd_msg.data[0]);
  publish("WRITE: " + String(0) + " - " + String(cmd_msg.data[0]));
  publish("WRITE: " + String(1) + " - " + String(cmd_msg.data[1]));
  drive_servos[0].write(cmd_msg.data[0]);
  drive_servos[1].write(cmd_msg.data[1]);

//publish("READ: " + String(0) + " - " + String(arm_servos[0].read()));
}

void update_drive_servos(const std_msgs::UInt16MultiArray& cmd_msg){
	for(int i = 0; i < sizeof(drive_servos); i++){
		drive_servos[i].write(cmd_msg.data[i]);
	}
}

// Subscribe to rostopic "arm_cmd" and "drive_cmd"
ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("arm_cmd", update_arm_servos);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive("drive_cmd", update_drive_servos);

void setup(){
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub_arm);
  nh.subscribe(sub_drive);
  drive_servos[0].attach(3); // Back wheel
  drive_servos[1].attach(5); // Front wheels
  arm_servos[0].attach(6);   // Arm Base
  arm_servos[1].attach(9);   // Arm Shoulder
  arm_servos[2].attach(10);  // Arm Elbow
  arm_servos[3].attach(11);  // Arm Wrist
}


void loop(){
  nh.spinOnce();
  delay(1);
}
