
#include <Arduino.h>
#include <pt.h>   // include protothread library


#define LEDPIN1 6
  

class arm_servo
{
public:
  arm_servo(int i2c_pwm_pin);
  ~arm_servo();

  void update();
  void setAngle();
  void setSpeed();
};

arm_servo::arm_servo();