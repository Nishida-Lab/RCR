#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ros/ros.h>
#include <rcr2018/TofFront.h>
#include <rcr2018/DcmCommand.h>
#include <rcr2018/LineCount.h>
#include <wiringPi.h>

bool is_finish {false}; //終了判定
const int dcm_pin {18}; //DCモータのPWMピン番号
const int arg_vel_max {500}; //最大角速度の指定
const int sig_a {2}; //シグモイド関数の定数
const int frequency {100}; //DCモータへの周波数
const int START_SW_PIN {25};

double sigmoid(double x) //シグモイド関数
{
  return 1 / (1 + std::exp(-sig_a * x));
}

double normalize_sig(double front_value) //シグモイド関数の正規化
{
  return 2 * (sigmoid(front_value) - 0.5);
}

//linecount callback function
void linemsgCallback(const rcr2018::LineCount::ConstPtr& msg) //フォトセンサーの回数読み込み
{
  if (msg->count > 3) //終了指示判定
  {
    is_finish = true;
  }
}


int main(int argc, char** argv)
{
  int pin_value {0};

  if (wiringPiSetupGpio() == -1)
  {
     std::cout << "ERROR" << std::endl;
     return -1;
  }

  pinMode(START_SW_PIN, INPUT);

  pin_value = digitalRead(START_SW_PIN);
  while(pin_value != 0)
  {
    pin_value = digitalRead(START_SW_PIN);
  }

  ros::init(argc, argv, "dcmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Publisher dcmotor_command_pub = nh.advertise<rcr2018::DcmCommand>("dcm_command", 1); //パブリッシャの設定

  rcr2018::DcmCommand dcm;

  ros::Subscriber dcmotor_command_sub {nh.subscribe<rcr2018::TofFront>("tof_front", 1,
    std::function<void (const rcr2018::TofFront::ConstPtr&)>
    {
      [&](const auto& constptr)
      {
        double target_value {0.0}; //目標値の初期化
        if (is_finish) //終了判定
        {
          target_value = 0.0;
        }
        else
        {
          //target_value = arg_vel_max * normalize_sig(constptr->front); //目標角速度の決定
          target_value = arg_vel_max * sigmoid(constptr->front); //目標角速度の決定

        }

        dcm.cmd_vel = target_value; //目標角速度をメッセージに代入

        dcmotor_command_pub.publish(dcm); //パブリッシュ
      }
    }
  )}; //tofサブスクライバの設定

  ros::Subscriber line_count_sub {nh.subscribe("line_count", 1, linemsgCallback)}; //linecountサブスクライバの設定

  ros::spin();

  return EXIT_SUCCESS;
}
