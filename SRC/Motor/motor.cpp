#include <iostream>
#include <cmath>
#include <stdexcept>

#include "motor.hpp"

using namespace std;

Motor::Motor() :
  rear_pwm_pin(1),
  rear_dir_pin(0),
  battery_volt(7.2),
  wheel_diameter(0.060)
{
  max_speed = (battery_volt/7.2) * (196.0/60.0) * (wheel_diameter*2*M_PI);

  if(wiringPiSetup() < 0)
    throw runtime_error("Motor constractor : faild wiringPiSetupGpio()");
  pinMode(rear_pwm_pin, PWM_OUTPUT);
  pinMode(rear_dir_pin, OUTPUT);

  cout << "Rear Motor Initialized" << endl;  

}

Motor::~Motor()
{
  digitalWrite(rear_dir_pin, LOW);
  pwmWrite(rear_pwm_pin, 0);

  cout << "Rear Motor Finalized" << endl;
}

void Motor::Drive_Rear(float speed)
{
  pwm_input = 1024 * speed/max_speed;
  
  if(pwm_input < 0) 
    digitalWrite(rear_dir_pin, HIGH);
  else
    digitalWrite(rear_dir_pin, LOW);
 
  pwmWrite(rear_pwm_pin, abs(pwm_input));
}

void Motor::Drive(float linear, float angular)
{
  Drive_Rear(linear);
}
