#include <iostream>
#include <cmath>
#include <pigpio.h>
#include <ros/ros.h>
#include <rcr2018/TofSide.h>
#include <rcr2018/SvmCommand.h>

const int PWMPIN_S {19}; //PWMピンのピン配置を19番ピンに
const int sig_a  {2}; //シグモイド関数の定数
const int frequency_sv {500}; //サーボモータの周波数

double sigmoid(double x) //シグモイド関数
{
  return 1 / (1 + std::exp(-sig_a * x));
}

double pulse_change_value(double difference_value) //パルス幅
{
  return 2 * 0.6 * (sigmoid(difference_value) - 0.5);
}

int main(int argc, char** argv)
{
  if (gpioInitialise() < 0) //pigpioの初期化
  {
    std::cout << "error" << std::endl;
    return -1;
  }

  gpioSetMode(PWMPIN_S, PI_OUTPUT); //PWMピンをセット

  ros::init(argc, argv, "svmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber svmotor_command_sub {nh.subscribe<rcr2018::TofSide>("tof_side", 1,
    std::function<void (const rcr2018::TofSide::ConstPtr&)>
    {
      [&](const rcr2018::TofSide::ConstPtr& constptr)
      {
        double pulse_target_value {0}; //目標角度の初期化

        double difference { constptr->left - constptr->right }; //左センサと右センサの値の差

        pulse_target_value = 1.5 - pulse_change_value(difference); //目標角度の決定

        double input_pwm_value { 1000000 * pulse_target_value / 20 }; //入力PWM信号のデューティ比を決定
        // pwmWrite(PWMPIN_S, input_pwm_value); //サーボモータに出力
        gpioHardwarePWM(PWMPIN_S, frequency_sv, static_cast<int>(input_pwm_value)); //RCサーボモータにPWM信号を入力
      }
    }
  )}; //サブスクライバの設定

  ros::spin();

  return 0;
}
