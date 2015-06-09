#ifndef __MOTOR__
#define __MOTOR__

#include <string>

using namespace std;

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
  float servo_max_ang;
  float servo_max_rate;
  float servo_min_ang;
  float servo_min_rate;
  float servo_input;
  float servo_space_ang;
  float servo_space_rate;
};

#endif
