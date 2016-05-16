#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

/*
	Singleton class, acting as the only instance of the Adafruit_PWMServoDriver...
*/
class I2C_PWM_Driver {
public:
	static Adafruit_PWMServoDriver* getInstance();	//Only one instance!
private:
  I2C_PWM_Driver() {}													// Hide constructors in order to
  I2C_PWM_Driver(I2C_PWM_Manager const&) {}		// prevent multiple instances
  I2C_PWM_Manager& operator=(I2C_PWM_Manager const&) {};
  static Adafruit_PWMServoDriver* pwm_driver;	// Pointer to the one instance
}

// Initialize the PWM Driver for the I2C_PWM_Driver class...
Adafruit_PWMServoDriver* I2C_PWM_Manager::pwm_driver = NULL;

/*
	Get instance of the PWM Driver
*/
Adafruit_PWMServoDriver* I2C_PWM_Manager::getInstance() {
	// If run for the first time
	if (!pwm_driver) {
		pwm_driver = new Adafruit_PWMServoDriver();
		pwm_driver.begin();					// Required
		pwm_driver.setPWMFreq(50);	// Default frequency = 50Hz
	}
	return pwm_driver;
}


/*
	Interface Class for Servos
*/
class I2C_PWM_Servo {
private:
	uint8_t pin;					// Current pin
	int angle;						// Current angle
	uint8_t speed;				// Current movement speed
	int angleToPulse();		// Convert angle to PWM
public:
	Servo();
	uint8_t attach(int pin_number);
	void detach();

	virtual void write(int angle);
	virtual void write(int angle, int speed);
	virtual int read();
	uint8_t attached();
}

class ArmServo : public I2C_PWM_Servo {
public:
	ArmServo();
private:
	const PROGMEM uint16_t SERVO_PULSE_MIN = 126;        // "-315 degrees", min pulse length count (out of 4096@50Hz)
	const PROGMEM uint16_t SERVO_PULSE_MAX = 504;        // "+315 degrees", max pulse length count (out of 4096@50Hz)
	const PROGMEM uint16_t SERVO_PULSE_FULL_TURN = 216;  // "360 degrees", one full rotation
	uint16_t SERVO_PULSE_NEUTRAL = 315;    // "0 degrees", center pulse length count (out of 4096@50Hz)
}

class DriveServo : public I2C_PWM_Servo {
public:
	DriveServo();
private:
	const PROGMEM uint16_t SERVO_PULSE_MIN = 126;        // "-315 degrees", min pulse length count (out of 4096@50Hz)
	const PROGMEM uint16_t SERVO_PULSE_MAX = 504;        // "+315 degrees", max pulse length count (out of 4096@50Hz)
	const PROGMEM uint16_t SERVO_PULSE_FULL_TURN = 216;  // "360 degrees", one full rotation
	uint16_t SERVO_PULSE_NEUTRAL = 315;    // "0 degrees", center pulse length count (out of 4096@50Hz)
}

class DCMotor {

}


