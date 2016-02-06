#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"

ros::NodeHandle  nh;
Servo arm_servoes[4];

void servo_cb( const std_msgs::UInt16MultiArray&  cmd_msg){

	for(int i=0;i<sizeof(arm_servoes);i++){
		
		arm_servoes[i].write(cmd_msg.data[i]);

	}

}
ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("arm_cmd", servo_cb);

void setup(){
nh.initNode();
nh.subscribe(sub_arm);
arm_servoes[0].attach(3);
arm_servoes[1].attach(5);
arm_servoes[2].attach(6);
arm_servoes[3].attach(9);

}


void loop(){
  nh.spinOnce();
  delay(1);
}
