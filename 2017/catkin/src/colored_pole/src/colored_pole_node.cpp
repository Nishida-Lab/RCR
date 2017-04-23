#include <tuple>
#include <vector>

#include <ros/ros.h>


namespace rcr {


class colored_pole
{
public:
  using color_type = std::tuple<int,int,int>;

  static constexpr color_type red    {0xff, 0x00, 0x00};
  static constexpr color_type yellow {0xff, 0xff, 0x00};
  static constexpr color_type blue   {0x00, 0x00, 0xff};
};


} // namespace rcr


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
