#include <iostream>
#include <cmath>
#include <stdexcept>

#include "motor.hpp"

using namespace std;

Motor::Motor() :
  rear_pwm_pin(26),
  rear_dir_pin(3),
  battery_volt(7.2),
  wheel_diameter(0.060)
{
  max_speed = (battery_volt/7.2) * (196.0/60.0) * (wheel_diameter*2*M_PI);

  if(wiringPiSetupGpio() < 0)
    throw runtime_error("Motor constractor : faild wiringPiSetupGpio()");
  pinMode(rear_pwm_pin, PWM_OUTPUT);
  pinMode(rear_dir_pin, OUTPUT);
}

Motor::~Motor()
{
  ;
}

void Motor::Drive_Rear(float speed)
{
  pwm_input = 1024 * speed/max_speed;
  
  if(pwm_input < 0) 
    digitalWrite(rear_dir_pin, LOW);
  else
    digitalWrite(rear_dir_pin, HIGH);

  pwmWrite(abs(rear_pwm_pin), pwm_input);
}

void Motor::Drive(float linear, float angular)
{
  Drive_Rear(linear);
}
