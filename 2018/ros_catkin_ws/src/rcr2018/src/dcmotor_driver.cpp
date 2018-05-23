#include <iostream>
#include <wiringPi.h>
#include <ros/ros.h>
#include <rcr2018/DcmCommand.h>
#include <rcr2018/AngVel.h>

const double kp = 0.0; //比例ゲインを決定
const double ki = 0.0; //積分ゲインを決定

const int PWMPIN_D = 18; //PWMピンのピン配置を18番ピンに
const int DIRPIN = 23; //DIRピンのピン配置を23番ピンに


double dev_tar_out = 0.0; //現時点での目標値と出力値の差の初期化
double dev_tar_out_pre = 0.0; //一つ前時点での目標値と出力値の差の初期化
double ang_vel = 0.0;

void commandmsgCallback(const rcr2018::DcmCommand::ConstPtr& msg)
{
   double target_value = msg->cmd_vel; //目標角速度

   double output_value = ang_vel; //出力角度

   dev_tar_out = target_value - output_value; //目標角速度と出力角度の差分

   double input_value = (kp * (dev_tar_out - dev_tar_out_pre)) + (ki * dev_tar_out); //PI制御器による入力値の決定

   int input_pwm_value = 0; //入力PWM信号のデューティ比を決定

   digitalWrite(DIRPIN, 0); //DIRピンの出力を決定
   pwmWrite(PWMPIN_D, input_pwm_value); //DCモータにPWM信号を入力
   dev_tar_out_pre = dev_tar_out; //次の時点のため、現時点での値を保存

}

void encodermsgCallback(const rcr2018::AngVel::ConstPtr& msg)
{
  ang_vel = msg->ang_vel;
}

int main(int argc, char** argv)
{
  if (wiringPiSetupGpio() == -1)
  {
    std::cout << "ERROR:Can't setup GPIO." << std::endl;
    return -1;
  }

  pinMode(PWMPIN_D, OUTPUT); //PWMピンのセット
  pinMode(DIRPIN, OUTPUT); //DIRピンのセット

  ros::init(argc, argv, "dcmotor_driver"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_driver_sub = nh.subscribe("dcm_command", 1, commandmsgCallback);
  ros::Subscriber encoder_sub = nh.subscribe("ang_vel", 1, encodermsgCallback);

  ros::spin();

  return 0;
}
