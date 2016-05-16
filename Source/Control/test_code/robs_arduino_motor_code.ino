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

// SPEED CHANGE DELAY
int const speed_change_delay = 10;

// ROS NODE HANDLE
ros::NodeHandle nh;

// MOTORS
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor * TEMP0 = AFMS.getMotor(1);
Adafruit_DCMotor * TEMP1 = AFMS.getMotor(2);
Adafruit_DCMotor * TEMP2 = AFMS.getMotor(3);
Adafruit_DCMotor * TEMP3 = AFMS.getMotor(4);

int current_speed_magnitude = 0;
uint8_t current_direction = RELEASE;

void stop() {
	while (current_speed_magnitude > 0) {
		// Decrement speed
		current_speed_magnitude--;
		// Set new speed on motors
		TEMP0->setSpeed(current_speed_magnitude);
 		TEMP1->setSpeed(current_speed_magnitude);
 		TEMP2->setSpeed(current_speed_magnitude);
 		TEMP3->setSpeed(current_speed_magnitude);
 		// Delay to avoid dangerous speed changes
 		delay(speed_change_delay);
 	}
 	TEMP0->run(RELEASE);
 	TEMP1->run(RELEASE);
 	TEMP2->run(RELEASE);
 	TEMP3->run(RELEASE);
 	current_direction = RELEASE;
} 

/*
	Sets motor speeds
*/
void setMotorSpeed(int new_speed_magnitude, uint8_t new_direction) {

	// CHECK IF THE ROVER SHOULD STOP
	// Will stop if speed is too big or too small
	if (new_speed_magnitude <= 0 || new_direction == RELEASE || new_speed_magnitude > 255) {
		stop();
		return;
	}

	// CHECK IF DIRECTION HAS CHANGED
	if (current_direction != new_direction) {
		stop();
		current_direction = new_direction;
	 	TEMP0->run(new_direction);
	 	TEMP1->run(new_direction);
	 	TEMP2->run(new_direction);
	 	TEMP3->run(new_direction);
	}
	
	// INCREASE TO DESIRED SPEED
	while (current_speed_magnitude < new_speed_magnitude) {
		current_speed_magnitude++;
		TEMP0->setSpeed(current_speed_magnitude);
		TEMP1->setSpeed(current_speed_magnitude);
		TEMP2->setSpeed(current_speed_magnitude);
		TEMP3->setSpeed(current_speed_magnitude);
		delay(speed_change_delay);
	}

	// DECREASE TO DESIRED SPEED
	while (current_speed_magnitude > new_speed_magnitude) {
		current_speed_magnitude--;
		TEMP0->setSpeed(current_speed_magnitude);
		TEMP1->setSpeed(current_speed_magnitude);
		TEMP2->setSpeed(current_speed_magnitude);
		TEMP3->setSpeed(current_speed_magnitude);
		delay(speed_change_delay);
	}
}

void motor_cb( const std_msgs::Int16& cmd_msg ){
	// INTERPRET COMMAND MESSAGE
	int new_speed;
	uint8_t new_direction;
	if (cmd_msg.data > 0) {					// Speed is greater than 0
		new_speed = cmd_msg.data;
		new_direction = FORWARD;
	} else if (cmd_msg.data < 0) {	// Speed is less than 0
		new_speed = -cmd_msg.data;
		new_direction = BACKWARD;
	} else {												// Speed is equal to 0
		new_speed = 0;
		new_direction = RELEASE;
	}
	setMotorSpeed(new_speed, new_direction);
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