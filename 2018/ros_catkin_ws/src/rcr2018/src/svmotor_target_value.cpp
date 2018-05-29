#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <ros/ros.h>
#include <rcr2018/TofSide.h>
#include <rcr2018/SvmCommand.h>

const int PWMPIN_S = 19; //PWMピンのピン配置を19番ピンに

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

  ros::Publisher svmotor_command_pub {n.advertise<rcr2018::SvmCommand>("svm_command", 1)}; //パブリッシャの設定

  rcr2018::SvmCommand svm;

  ros::Subscriber svmotor_command_sub {nh.subscribe("tof_side", 1, 
    std::function<void (const rcr2018::TofSide::ConstPtr&)>
    {
      [&](const rcr2018::TofSide::ConstPtr& constptr)
      {
        double target_value = 0; //目標角度の初期化

        double difference = constptr->left - constptr->right; //左センサと右センサの値の差

        target_value = 0.6 * std::tanh(difference); //目標角度の決定

        int input_pwm_value = target_value; //入力PWM信号のデューティ比を決定
        // pwmWrite(PWMPIN_S, input_pwm_value); //サーボモータに出力
        svm.cmd_ang_vel = target_value;
        svmotor_command_pub.publish(svm);
      }
    }
  )}; //サブスクライバの設定

  ros::spin();

  return 0;
}
