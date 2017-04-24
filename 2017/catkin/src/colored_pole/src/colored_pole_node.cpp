#include <tuple>
#include <utility>
#include <vector>

#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <colored_pole/existence.h>


bool return_true(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return true;
}


auto lambda = [&](std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) { return true; };


namespace rcr {


class functor
{
public:
  using req_type = ::colored_pole::existence::Request;
  using res_type = ::colored_pole::existence::Response;

  bool operator()(req_type& req, res_type& res) noexcept
  {
    return true;
  }
};


class colored_pole
{
public:
  using color_type = std::tuple<int,int,int>;
  using coord_type = std::pair<double, double>;

  static constexpr color_type red    {0xff, 0x00, 0x00};
  static constexpr color_type yellow {0xff, 0xff, 0x00};
  static constexpr color_type blue   {0x00, 0x00, 0xff};

  static constexpr double window_width  {500};
  static constexpr double window_height {500};

private:
  const coord_type coord_;

  // ros::ServiceServer server_;

public:
  colored_pole(const ros::NodeHandle& node_handle, coord_type&& coord)
    : coord_ {check(std::forward<decltype(coord)>(coord))}
      // server_ {node_handle.advertiseService("existence", &colored_pole::callback, this)}
  {
    // server_ = node_handle.advertiseService<functor::req_type, functor::res_type>("existence", functor());
    // server_ = node_handle.advertiseService("existence", return_true);
    // server_ = node_handle.advertiseService("existence", lambda);
  }

private:
  static coord_type check(coord_type&& coord)
  {
    coord.first = (coord.first < 0 ? 0 : (window_width < coord.first ? window_width : coord.first));
    coord.second = (coord.second < 0 ? 0 : (window_height < coord.second ? window_height : coord.second));
    return std::forward<decltype(coord)>(coord);
  }

  // bool callback(::colored_pole::existence::Request&  request,
  //               ::colored_pole::existence::Response& response)
  // {
  //   auto a {std::pow(coord_.first - request.x, 2)};
  //   auto b {std::pow(coord_.second - request.y, 2)};
  //
  //   if (std::sqrt(a + b) < request.range) { return true; }
  //   else { return false; }
  // }
};


} // namespace rcr


int main(int argc, char** argv)
{
  ros::init(argc, argv, "colored_pole_node");

  ros::NodeHandle node_handle {"~"};

  std::vector<rcr::colored_pole> poles {
    {node_handle, {100.0, 100.0}},
    {node_handle, {200.0, 200.0}}
  };

  while (ros::ok())
  {
    ROS_INFO_STREAM(ros::this_node::getName());
  }

  return 0;
}
