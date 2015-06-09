#include <wiringPi.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <unistd.h>

#include "motor.hpp"

Motor::Motor() :
  rear_pwm_pin(23),
  rear_dir_pin(22),
  battery_volt(7.2),
  wheel_diameter(0.060),
  wheel_base(0.160),
  servo_pin(1),
  servo_left_ang(0.410),
  servo_left_rate(69),
  servo_right_ang(-0.410),
  servo_right_rate(92),
  motor_geer(60),
  encoder_geer(50),
  diff_geer(54),
  left_wheel_geer(36)
{
  // Initialize WPin
  if(wiringPiSetup() < 0)
    throw runtime_error("Wiring Pi : Can not initialize ...orz");

  // WPin 22(BCM 6) to OUTPUT Pin (DC Motor)
  pinMode(rear_dir_pin, OUTPUT);
  digitalWrite(rear_dir_pin, LOW);

  // WPin 23(BCM 13) to PWM Pin (DC Motor)
  pinMode(rear_pwm_pin, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetClock(400);
  pwmSetRange(1024);

  // WPin 24(BCM 19) to PWM Pin (Servo)
  pinMode(servo_pin, PWM_OUTPUT);
  pwmSetMode(PWM_MODE_MS);
  pwmSetClock(400);
  pwmSetRange(1024);

  max_speed = (battery_volt/7.2) * ((196.0/60.0)*(36.0/10.0)) * (wheel_diameter*M_PI);

  servo_space_ang = servo_left_ang - servo_right_ang;
  servo_space_rate = servo_right_rate - servo_left_rate;
  servo_middle_rate = (servo_right_rate + servo_left_rate) / 2.0;

  Drive(0,0);
  usleep(500000);

  cout << "Initialize Servo & DC Motor Control ..." << endl;
}

Motor::~Motor()
{
  // Finalize WPin
  Drive(0,0);
  cout << "Finalize Motor & Servo" << endl;
}

// speed [rad/s]
void Motor::Drive_Rear(float speed)
{
  float pwm_input = 1024 * speed/max_speed;

  if(pwm_input < 0)
    digitalWrite(rear_dir_pin, HIGH);
  else
    digitalWrite(rear_dir_pin, LOW);
	      
  pwmWrite(rear_pwm_pin, abs(pwm_input));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Publically, To Control Servo Motor
// We need to make the pulse width from 0.1 [ms] to 2.0 [ms]. (May be from 0.5 [ms] to 2.5 [ms].)
// And the Control Time is from 10 [ms] to 20 [ms].
// At the MiniS RB955b, this Servo Motor can move
// from -70 [deg] to 70 [deg]. (The Machiyo machine can move from -23 [deg] to 23 [deg].)
// Additional info : the speed is 0.20 [s/60[deg]] on 4.8 [V].
//////////////////////////////////////////////////////////////////////////////////////////////////
void Motor::Drive_Front(float angular)
{
  float pwm_input = servo_left_rate + ( (servo_left_ang - angular)/servo_space_ang * servo_space_rate );

  cout << angular << endl;
  cout << pwm_input << endl;

  pwmWrite(servo_pin, pwm_input);
}

void Motor::Drive(float linear, float angular)
{
  if(!linear)
    Drive_Front(0);
  else
    Drive_Front(atan(wheel_base*angular/linear));
  
  Drive_Rear(linear);
}
