/* First motoro control test sketch
 * 2/5/16
 * 
 * This will subscribe to a /motor_speed topic in ROS and run
 * 7 motors. 4 will be through the AdaFruit motorshield library 
 * and 3 will be driven by a PWM signal from the arduino itself. 
 * These 3 will drive transistors in an H-bridge configuration
 * to actually drive the motors.
 */
 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif


#include <Servo.h> 
#include <ros.h>
#include <inttypes.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor * TEMP0 = AFMS.getMotor(1);
Adafruit_DCMotor * TEMP1 = AFMS.getMotor(2);
Adafruit_DCMotor * TEMP2 = AFMS.getMotor(3);
Adafruit_DCMotor * TEMP3 = AFMS.getMotor(4);

void motor_cb( const std_msgs::Int16& cmd_msg ){
	// Speed and direction variables
	int SPEED;
	uint8_t DIRECTION;

	// Interpret "cmd_msg"
	if (cmd_msg.data == 0) {
		SPEED = 0;
		DIRECTION = RELEASE;
	} else if (cmd_msg.data > 0) {
		SPEED = cmd_msg.data;
		DIRECTION = FORWARD;
	} else {
		SPEED = -cmd_msg.data;
		DIRECTION = BACKWARD;
	}

	TEMP0->setSpeed(SPEED);
 	TEMP0->run(DIRECTION);
 	TEMP1->setSpeed(SPEED);
 	TEMP1->run(DIRECTION);
 	TEMP2->setSpeed(SPEED);
 	TEMP2->run(DIRECTION);
 	TEMP3->setSpeed(SPEED);
 	TEMP3->run(DIRECTION);
} 

ros::Subscriber<std_msgs::Int16> sub_drive_speed("drive_speed", motor_cb);

void setup() {
  
  AFMS.begin();
  
  nh.initNode();
  nh.subscribe(sub_drive_speed);
  
 	TEMP0->run(RELEASE);
 	TEMP1->run(RELEASE);
 	TEMP2->run(RELEASE);
 	TEMP3->run(RELEASE);
  
}


void loop() {
  nh.spinOnce();
  delay(1);
}