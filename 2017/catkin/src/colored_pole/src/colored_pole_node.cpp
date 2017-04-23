#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "colored_pole_node");

  ros::NodeHandle node_handle {"~"};

  while (ros::ok())
  {
    ROS_INFO_STREAM(ros::this_node::getName());
  }

  return 0;
}
