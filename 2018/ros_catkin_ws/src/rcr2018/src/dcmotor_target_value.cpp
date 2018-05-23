#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ros/ros.h>
#include <rcr2018/TofFront.h>
#include <rcr2018/DcmCommand.h>
#include <rcr2018/LineCount.h>

bool is_finish = false;
const int dcm_pin = 18;

//linecount callback function
void linemsgCallback(const rcr2018::LineCount::ConstPtr& msg)
{
  if (msg->count > 3)
  {
    is_finish = true;
  }
}


int main(int argc, char** argv)
{
  const int frequency = 100;

  ros::init(argc, argv, "dcmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Publisher dcmotor_command_pub = nh.advertise<rcr2018::DcmCommand>("dcm_command", 1); //パブリッシャの設定

  rcr2018::DcmCommand dcm;

  ros::Subscriber dcmotor_command_sub {nh.subscribe<rcr2018::TofFront>("tof_front", 1, 
    std::function<void (const rcr2018::TofFront::ConstPtr&)>
    {
      [&](const auto& constptr)
      {
        double target_value = 0.0; //目標値の初期化
        if (is_finish)
        {
        target_value = 0.0;
        }
        else
        {
          target_value = 1 * std::tanh(constptr->front); //目標角速度の決定
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
