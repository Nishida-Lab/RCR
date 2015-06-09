#ifndef __MOTOR__
#define __MOTOR__

using namespace std;

class Motor
{
public:
  
  Motor();
  ~Motor();

  void Drive(float linear, float angular);

  int motor_geer;
  int encoder_geer;
  int diff_geer;
  int left_wheel_geer;
  float battery_volt;
  float wheel_diameter;
  float wheel_base;
  float tread;


private:
  
  void Drive_Rear(float speed);
  void Drive_Front(float angular);

  // Pin number
  int rear_pwm_pin;
  int rear_dir_pin;
  int servo_pin;

  // Max speed of the DC Motor
  float max_speed;

  // for Servo 
  float servo_left_ang;
  float servo_left_rate;
  float servo_right_ang;
  float servo_right_rate;
  float servo_space_ang;
  float servo_space_rate;
  float servo_middle_rate;
};

#endif
