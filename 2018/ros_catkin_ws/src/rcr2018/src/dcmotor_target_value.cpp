#include "ros/ros.h"
#include "rcr2018/TofFront.h"
#include "rcr2018/DcmCommand.h"

ros::NodeHandle n;

ros::publisher dcmotor_command_pub = n.advertise<rcr2018::DcmCommand>("dcmotor_command_msg", 1);

rcr2018::DcmCommand dcm;

//メッセージを受信したとき動作する関数
void msgCallback(const rcr2018::TofFront::ConstPtr& msg)
{
  double target_value = msg.front; //目標値の決定

  dcm.cmd_vel = target_value;

  dcmotor_command_pub.publish(dcm);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dcmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_command_sub = nh.subscribe("tof_front_msg", 1, msgCallback);

  ros::spin();

  return 0;
}
