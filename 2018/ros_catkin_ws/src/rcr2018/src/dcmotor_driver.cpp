#include <pigpio.h>
#include "ros/ros.h"
#include "rcr2018/DcmCommand.h"
#include "rcr2018/AngVel.h"

#define kp 0;
#define ki 0;

void msgCallback(const rcr2018::DcmCommand::ConstPtr& msgt, const rcr2018::AngVel::ConstPtr& msgr)
{ 
   double target_value = msgt.cmd_vel;

   double output_value = msgr.ang_vel;

   double dev_tar_out = target_value - output_value;

   double input_value = kp * dev_tar_out + ki * dev_tar_out;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dcmotor_driver"); //ノード名の初期化

  ros::NodeHandle nh; //ノードハンドル宣言

  ros::Subscriber dcmotor_driver_sub = nh.subscribe("dcmotor_driver_msg", 1, msgCallback);

  ros::spin();

  return 0;
}
