#ifndef __MOTOR__
#define __MOTOR__

#include <wiringPi.h>

class Motor
{
public:
  
  Motor();
  ~Motor();

  void Drive(float linear, float angular);

private:
  
  void  Drive_Rear(float speed);  

  int rear_pwm_pin;
  int rear_dir_pin;

  float max_speed;
  float battery_volt;
  float wheel_diameter;
  int pwm_input;
};

#endif
