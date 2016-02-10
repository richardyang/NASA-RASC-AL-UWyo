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
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *Front_L = AFMS.getMotor(1);
Adafruit_DCMotor *Front_R = AFMS.getMotor(2);
Adafruit_DCMotor *Rear = AFMS.getMotor(4);
//Adafruit_DCMotor *Front_L = AFMS.getMotor(4);
//Adafruit_DCMotor *Front_L = AFMS.getMotor(1);
//Adafruit_DCMotor *Front_L = AFMS.getMotor(1);

void motor_cb( const std_msgs::UInt16& cmd_msg ){
 Front_L->setspeed(cmd_msg.data);
 Front_R->setspeed(cmd_msg.data);
 Rear->setspeed(cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub("motor_speed", motor_cb);

void setup() {

  AFMS.begin();

  nh.initNode();
  nh.subscribe(sub);

  Front_L->run(RELEASE);
  Front_R->run(RELEASE);
  Rear->run(RELEASE);

}


void loop() {
  nh.spinOnce();
  delay(1);
}
