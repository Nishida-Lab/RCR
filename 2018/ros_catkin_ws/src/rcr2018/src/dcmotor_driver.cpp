#include <wiringPi.h>
#include "ros/ros.h"
#include "rcr2018/DcmCommand.h"
#include "rcr2018/AngVel.h"

#define kp 0; //比例ゲインを決定
#define ki 0; //積分ゲインを決定

const int PWMPIN_D 18; //PWMピンのピン配置を18番ピンに
const int DIRPIN   23; //DIRピンのピン配置を23番ピンに

pinMode(PWMPIN_D, OUTPUT); //PWMピンのセット
pinMODE(DIRPIN, OUTPUT); //DIRピンのセット

double dev_tar_out = 0.0; //現時点での目標値と出力値の差の初期化
double dev_tar_out_pre = 0.0; //一つ前時点での目標値と出力値の差の初期化

void msgCallback(const rcr2018::DcmCommand::ConstPtr& msgt, const rcr2018::AngVel::ConstPtr& msgr)
{
   double target_value = msgt.cmd_vel; //目標角速度

   double output_value = msgr.ang_vel; //出力角度

   dev_tar_out = target_value - output_value; //目標角速度と出力角度の差分

   double input_value = kp * (dev_tar_out - dev_tar_out_pre) + ki * dev_tar_out; //PI制御器による入力値の決定

   int input_pwm_value = 0; //入力PWM信号のデューティ比を決定

   digitalWrite(DIRPIN, 0); //DIRピンの出力を決定
   pwmWrite(PWMPIN_D, input_pwm_value); //DCモータにPWM信号を入力
   dev_tar_out_pre = dev_tar_out; //次の時点のため、現時点での値を保存

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dcmotor_driver"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_driver_sub = nh.subscribe("dcmotor_driver_msg", 1, msgCallback);

  ros::spin();

  return 0;
}
