#ifndef INCLUDED_ROBOCAR_DRIVER_RADIO_CONTROL_HPP_
#define INCLUDED_ROBOCAR_DRIVER_RADIO_CONTROL_HPP_


#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <utilib/unique_fd.hpp>


namespace robocar {


enum class button : std::size_t
{
  select          =  0,
  stick_left      =  1,
  stick_right     =  2,
  start           =  3,
  cross_up        =  4,
  cross_right     =  5,
  cross_down      =  6,
  cross_left      =  7,
  rear_left_2     =  8,
  rear_right_2    =  9,
  rear_left_1     = 10,
  rear_right_1    = 11,
  action_triangle = 12,
  action_circle   = 13,
  action_cross    = 14,
  action_square   = 15,
  pairing         = 16,
};


enum class axis : std::size_t
{
  stick_left_leftwards   =  0,
  stick_left_upwards     =  1,
  stick_right_leftwards  =  2,
  stick_right_upwards    =  3,
  button_cross_up        =  4,
  button_cross_right     =  5,
  button_cross_down      =  6,
  button_cross_left      =  7,
  button_rear_left_2     =  8,
  button_rear_right_2    =  9,
  button_rear_left_1     = 10,
  button_rear_right_1    = 11,
  button_action_triangle = 12,
  button_action_circle   = 13,
  button_action_cross    = 14,
  button_action_square   = 15,
  accelerometer_left     = 16,
  accelerometer_forward  = 17,
  accelerometer_up       = 18,
  gyro_yaw               = 19,
};


class radio_controler
{
  utilib::unique_fd fd_;
  js_event event_;

public:
  std::vector<std::int8_t> button;
  std::vector<std::int16_t> axis;

  radio_controler(const std::string& device_name)
    : fd_ {open(device_name.c_str(), O_RDONLY)}
  {
    if (!fd_)
    {
      std::cerr << "[error] failed to open " << device_name << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::size_t size_buffer {0};

    ioctl(fd_, JSIOCGAXES, &size_buffer);
    button.resize(size_buffer, 0);

    ioctl(fd_, JSIOCGBUTTONS, &size_buffer);
    axis.resize(size_buffer, 0);

    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  auto& update()
  {
    read(fd_, &event_, sizeof event_);

    switch (event_.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_BUTTON:
      button[static_cast<std::size_t>(event_.number)] = event_.value;
      break;

    case JS_EVENT_AXIS:
      axis[static_cast<std::size_t>(event_.number)] = event_.value;
      break;
    }

    return *this;
  }
};


  // for (robocar::radio_controler ps3joy {"/dev/input/js0"}; ;)
  // {
  //   ps3joy.update();
  //
  //   robocar::vector<double> direction {
  //     ps3joy.axis[0] / static_cast<double>(std::numeric_limits<std::int16_t>::max()),
  //    -ps3joy.axis[1] / static_cast<double>(std::numeric_limits<std::int16_t>::max())
  //   };
  //
  //   driver.write(direction, 0.18, 0.5);
  // }


} // namespace robocar


#endif
