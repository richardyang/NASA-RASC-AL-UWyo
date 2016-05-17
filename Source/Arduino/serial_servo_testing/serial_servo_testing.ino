// serial_motor_testing.ino
// Author:  John Ross Petrutiu
// Date:    May 17, 2016

/* Description:
  This program lets the user send PWM values over serial in order to test for a 
  servo's PWM range when using Adafruit's 16 channel, 12 bit PWM breakout board
*/

// Include required libraries for Adafruit PWM board
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Declare Adafruit PWM baord object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PWM values for Adafruit board
const int PWM_MIN     = 0;
const int PWM_NEUTRAL = 310;
const int PWM_MAX     = 4096;

// ---------------------------------------------------
//       CHANGE THIS ONE TO TEST A DIFFERENT PIN      
// PWM servo pin
const uint8_t PWM_PIN = 3;
// ---------------------------------------------------

void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("Setup Started...");

  // Initialize Adafruit PWM Board (50 Hz yo...)
  pwm.begin();
  pwm.setPWMFreq(50);

  Serial.println("PWM board Initialized...");

  // Set all servos to approximate, default "neutral"
  for (int servoNumber = 0; servoNumber <= 15; servoNumber ++) {
    pwm.setPWM(servoNumber, 0, PWM_NEUTRAL);
  }

  Serial.print("All PWM pins set to neutral (");
  Serial.print(PWM_NEUTRAL);
  Serial.println(")...");
}

void loop() {
  // Check if serial connection is available
  while (Serial.available() > 0) {
    // Look for valid input from serial
    int pwmValue = Serial.parseInt();

    // Look for new line (end of message)
    if (Serial.read() == '\n') {
      // Constrain pwmValue from 0 to 255
      pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);

      // Write pwmValue to led
      pwm.setPWM(PWM_PIN, 0, pwmValue);

      // Print pwmValue back to serial (debug)
      Serial.println(pwmValue);
    }
  }
}