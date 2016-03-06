/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define ARM_SERVO_MIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define ARM_SERVO_MAX  505 // this is the 'maximum' pulse length count (out of 4096)
#define ARM_SERVO_NEUTRAL 315
#define ARM_SERVO_FULL_TURN 216

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  //Serial.println("16 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(servonum, 0, (uint16_t)(ARM_SERVO_MIN + ((ARM_SERVO_MAX - ARM_SERVO_MIN)/2)));
  //delay(15000);
}

/* ### arm_angle_to_pulse() ###
  Converts a servo angle (0 - 180) into a PWM pulse
  @INPUT int - new arm servo angle
  @RETURN double - pwm pulse
*/
uint16_t arm_angle_to_pulse(int int_angle) {
  // Typecast angle to double to prevent overflow
  double angle = int_angle * 1.0;

  // PWM = <neurtal_pwm> + (angle/360) * <full_turn_pwm>
  uint16_t pulse = (uint16_t) (ARM_SERVO_NEUTRAL + ((angle*ARM_SERVO_FULL_TURN)/360));

  // Check if pulse is a sentinel, or in allowed range
  if (pulse < ARM_SERVO_MIN) {
    pulse = (uint16_t) ARM_SERVO_MIN;
  } else if (pulse > ARM_SERVO_MAX) { 
    pulse = (uint16_t) ARM_SERVO_MAX;
  }

  return pulse;
}

void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
    int angle_read = Serial.readString().toInt();
    Serial.println(angle_read);

    uint16_t pulse = arm_angle_to_pulse(angle_read);
    Serial.println((int)pulse);
    
    pwm.setPWM(servonum, 0, pulse);
  }
}
