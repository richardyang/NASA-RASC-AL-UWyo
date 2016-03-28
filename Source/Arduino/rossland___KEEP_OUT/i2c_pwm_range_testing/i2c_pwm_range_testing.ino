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

//#define SERVO_MIN  126      // this is the 'minimum' pulse length count (out of 4096@50Hz)
//#define SERVO_MAX  504      // this is the 'maximum' pulse length count (out of 4096@50Hz)

#define SERVO_MIN  200      // this is the 'minimum' pulse length count (out of 4096@50Hz)
#define SERVO_MAX  504      // this is the 'maximum' pulse length count (out of 4096@50Hz)

#define SERVO_NEUTRAL 315   // this is the '90 degree' pulse length count (out of 4096@50Hz)
#define SERVO_FULL_TURN 216 // this is thie pulse length count delta for '360 degrees' (4096@50Hz)



// our servo # counter
uint8_t servonum = 5;

void setup() {

  
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Digital servos run at ~50 Hz updates
  pwm.setPWM(4, 0, 295);
  pwm.setPWM(5, 0, 295);
  pwm.setPWM(6, 0, 295);
  // PWM LIMITS FOR STEER SERVOS

  // MIN = 105
  // MAX = 485
  // DEAD CENTER = 295
  // 90 DEGREE TURN = 179 (~180)
  // 360 DEGREE TURN = 720



// ~260

}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // 50 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  // Drive each servo one at a time
  /*
  Serial.println(servonum);
  for (uint16_t pulselen = SERVO_MIN; pulselen < SERVO_MAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(2000);
  for (uint16_t pulselen = SERVO_MAX; pulselen > SERVO_MIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }
  */

  delay(2000);
}
