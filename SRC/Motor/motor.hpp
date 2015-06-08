#ifndef __MOTOR__
#define __MOTOR__

#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <stdexcept>
#include <pstreams/pstreams.h>

using namespace std;
using namespace redi;

class Motor
{
public:
  
  Motor();
  ~Motor();

  void Drive(float linear, float angular);

private:
  
  void Drive_Rear(float speed);
  void Drive_Front(float angular);

  int rear_pwm_pin;
  int rear_dir_pin;

  float max_speed;
  float battery_volt;
  float wheel_diameter;
  int pwm_input;

  string servo_pipe;
  opstream servo_pout;
  float servo_max_ang;
  float servo_max_rate;
  float servo_min_ang;
  float servo_min_rate;
  float servo_input;
  float servo_space_ang;
  float servo_space_rate;
};

#endif
