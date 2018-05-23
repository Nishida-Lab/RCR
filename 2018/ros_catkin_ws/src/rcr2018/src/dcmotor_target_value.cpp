#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <rcr2018/TofFront.h>
#include <rcr2018/DcmCommand.h>
#include <rcr2018/LineCount.h>

ros::NodeHandle n; //パブリッシャのノードハンドル宣言

ros::Publisher dcmotor_command_pub = n.advertise<rcr2018::DcmCommand>("dcm_command", 1); //パブリッシャの設定

rcr2018::DcmCommand dcm;

bool is_finish = false;

//TofFrontメッセージを受信したとき動作する関数
void tofmsgCallback(const rcr2018::TofFront::ConstPtr& msg)
{
  double target_value = 0.0; //目標値の初期化
  if (is_finish)
  {
  target_value = 0.0;
  }
  else
  {
    target_value = 0 * std::tanh(msg->front); //目標角速度の決定
  }

  dcm.cmd_vel = target_value; //目標角速度をメッセージに代入

  dcmotor_command_pub.publish(dcm); //パブリッシュ

}

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
  ros::init(argc, argv, "dcmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_command_sub = nh.subscribe("tof_front", 1, tofmsgCallback); //tofサブスクライバの設定

  ros::Subscriber line_count_sub = nh.subscribe("line_count", 1, linemsgCallback); //linecountサブスクライバの設定

  ros::spin();

  return 0;
}
