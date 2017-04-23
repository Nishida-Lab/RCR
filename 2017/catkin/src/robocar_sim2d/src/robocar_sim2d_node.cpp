#include <string>

#include <ros/ros.h>
#include <turtlesim/turtle_frame.h>

#include <QApplication>


namespace turtlesim {

class TurtleApp
  : public QApplication
{
public:
  ros::NodeHandlePtr node_handle_; // boost::shared_ptr<ros::NodeHandle>

  using base_class = QApplication;

  explicit TurtleApp(int argc, char** argv, const std::string& node_name)
    : base_class {argc, argv}
  {
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    node_handle_.reset(new ros::NodeHandle);
  }

  auto exec()
  {
    turtlesim::TurtleFrame frame {};
    frame.show();

    return base_class::exec();
  }
};

} // namespace turtlesim


int main(int argc, char** argv)
{
  turtlesim::TurtleApp app {argc, argv, "robocar_sim2d_node"};

  return app.exec();
}
