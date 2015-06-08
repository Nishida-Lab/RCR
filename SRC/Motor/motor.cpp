#include "motor.hpp"

Motor::Motor() :
  rear_pwm_pin(1),
  rear_dir_pin(0),
  battery_volt(7.2),
  wheel_diameter(0.060),
  servo_pipe("/dev/servoblaster"),
  servo_max_ang(0.410),
  servo_max_rate(65),
  servo_min_ang(-0.410),
  servo_min_rate(40)
{
  // -----------------------------------------------------------
  // Rear Motor
  // -----------------------------------------------------------
  // 電圧とタイヤ経を元に、理論上の最高速度[m/s]を算出
  // 196[rpm] <- 3633K36(1:36)より、実際は3633K10(1:10)
  // -----------------------------------------------------------
  max_speed = (battery_volt/7.2) * ((196.0/60.0)*(36.0/10.0)) * (wheel_diameter*M_PI);

  if(wiringPiSetup() < 0)
    throw runtime_error("Motor constractor : faild wiringPiSetupGpio()");
  pinMode(rear_pwm_pin, PWM_OUTPUT);
  pinMode(rear_dir_pin, OUTPUT);

  cout << "Rear Motor Initialized" << endl;  

  // -----------------------------------------------------------
  // Front Motor
  // -----------------------------------------------------------
  // 詳しくは、Servoblastで検索すること。
  // -----------------------------------------------------------
  servo_pout.open(servo_pipe.c_str());
  if(!servo_pout)
    throw runtime_error("Motor constractor : faild to open servoblast");
  
  servo_space_ang = servo_max_ang - servo_min_ang;
  servo_space_rate = servo_max_rate - servo_min_rate;
 
  cout << "Front Motor Initialized" << endl;
}

Motor::~Motor()
{
  digitalWrite(rear_dir_pin, LOW);
  pwmWrite(rear_pwm_pin, 0);
  cout << "Rear Motor Finalized" << endl;

  servo_pout.close();
  cout << "Front Motor Finalized" << endl;
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

void Motor::Drive_Front(float angular)
{
  if(!servo_pout.is_open())
    throw runtime_error("Drive_Front() : servoblast not opened");

  servo_input = servo_max_rate - (((servo_max_ang-angular)/servo_space_ang)*servo_space_rate);
  
  servo_pout << "0=" << servo_input << "%" << endl;

  cout << servo_input << endl;
}

void Motor::Drive(float linear, float angular)
{
  Drive_Rear(linear);
  Drive_Front(angular);
}
