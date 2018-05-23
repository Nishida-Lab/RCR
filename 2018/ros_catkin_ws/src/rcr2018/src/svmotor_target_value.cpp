#include "ros/ros.h"
#include "rcr2018/TofSide.h"
#include "rcr2018/SvmCommand.h"

ros::NodeHandle n;

ros::publisher svmotor_command_pub = n.advertise<rcr2018::SvmCommand>("svmotor_command_msg", 1);

rcr2018::SvmCommand svm;

//メッセージを受信したとき動作する関数
void msgCallback(const rcr2018::TofSide::ConstPtr& msg)
{
  double target_value = msg.left - msg.right; //目標値の決定

  svm.cmd_ang_vel = target_value;

  svmotor_command_pub.publish(svm);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "svmotor_target_value"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber svmotor_command_sub = nh.subscribe("tof_side_msg", 1, msgCallback);

  ros::spin();

  return 0;
}
