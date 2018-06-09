#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <ros/ros.h>
#include <rcr2018/TofSide.h>
#include <rcr2018/SvmCommand.h>

const int PWMPIN_S = 19; //PWMピンのピン配置を19番ピンに
const int sig_a = 2;

double sigmoid(double x)
{
  double sig_value =  1 / (1 + std::exp(-sig_a * x));
  return sig_value;
}

double pulse_change_value(double difference_value)
{
  double change_value = 2 * 0.6 * (sigmoid(difference_value) - 0.5);
  return change_value;
}

int main(int argc, char** argv)
{
  if (wiringPiSetupGpio() == -1)
  {
  std::cout << "ERROR:Can't setup GPIO." << std::endl;
  return -1;
  }

  pinMode(PWMPIN_S, OUTPUT); //PWMピンをセット

  ros::init(argc, argv, "svmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber svmotor_command_sub {nh.subscribe<rcr2018::TofSide>("tof_side", 1,
    std::function<void (const rcr2018::TofSide::ConstPtr&)>
    {
      [&](const rcr2018::TofSide::ConstPtr& constptr)
      {
        double pulse_target_value = 0; //目標角度の初期化

        double difference = constptr->left - constptr->right; //左センサと右センサの値の差

        pulse_target_value = pulse_change_value(difference); //目標角度の決定

        int input_pwm_value = pulse_target_value; //入力PWM信号のデューティ比を決定
        // pwmWrite(PWMPIN_S, input_pwm_value); //サーボモータに出力
        pwmWrite(PWMPIN_S, input_pwm_value); //RCサーボモータにPWM信号を入力
      }
    }
  )}; //サブスクライバの設定

  ros::spin();

  return 0;
}
