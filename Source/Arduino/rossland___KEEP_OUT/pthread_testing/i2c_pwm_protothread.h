#include <Arduino.h>
#include <pt.h>   // include protothread library

#include <Wire.h>                     // For I2C
#include <Adafruit_PWMServoDriver.h>  // For I2C PWM board

class i2c_pwm_protothread {
	public:
		i2c_pwm_protothread();
		~i2c_pwm_protothread();

		static init();
		
	private:
		static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
};

Adafruit_PWMServoDriver i2c_pwm_protothread::servoDriver = Adafruit_PWMServoDriver();




// outer.hpp

class outer
{
public:
  class Inner;
  outer();
  void function1();
  Inner function2();
};
// outer_inner.hpp

#include "outer.hpp"

class outer::Inner
{
  void func();
};
Then, in the implementation:

#include "outer.hpp"
#include "outer_inner.hpp"

void outer::bar() { /* ... */ }
void outer::Inner::func() { /* ... */ }