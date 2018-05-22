#include <cmath>
#include "ros/ros.h"
#include "rcr2018/TofFront.h"
#include "rcr2018/DcmCommand.h"
#include "rcr2018/LineCount.h"

ros::NodeHandle n; //パブリッシャのノードハンドル宣言

ros::publisher dcmotor_command_pub = n.advertise<rcr2018::DcmCommand>("dcmotor_command_msg", 1); //パブリッシャの設定

rcr2018::DcmCommand dcm;

//メッセージを受信したとき動作する関数
void msgCallback(const rcr2018::TofFront::ConstPtr& msgt, const rcr2018::LineCount& msgl)
{
  double target_value = 0.0; //目標値の初期化

  if(msgl.count==4) //ラインを読んだ回数が４なら実行
  {
    target_value = 0.0; //目標角速度を０に決定
  }
  else
  {
    target_value = 0 * std::tanh(msgt.front); //目標角速度の決定
  }

  dcm.cmd_vel = target_value; //目標角速度をメッセージに代入

  dcmotor_command_pub.publish(dcm); //パブリッシュ

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dcmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_command_sub = nh.subscribe("tof_front_msg", 1, msgCallback); //サブスクライバの設定

  ros::spin();

  return 0;
}
