#include <iostream>
#include <iomanip>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"
const std::string joy_dev {"/dev/input/js0"};


int main()
{
  int joy_fd(-1), num_of_axis(0), num_of_buttons(0);
  char name_of_joystick[80];

  std::vector<char> joy_button;
  std::vector<int>  joy_axis;

  if((joy_fd = open(joy_dev.c_str(), O_RDONLY)) < 0)
  {
    std::cerr << "[error] failed to open " << joy_dev << std::endl;
    return -1;
  }

  ioctl(joy_fd, JSIOCGAXES,     &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS,  &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  joy_button.resize(num_of_buttons, 0);
  joy_axis.resize(num_of_axis, 0);

  std::cout << "[debug] joystick: " << name_of_joystick << std::endl
            << "            axis: " << num_of_axis      << std::endl
            << "         buttons: " << num_of_buttons   << std::endl;

  fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode

  while (true)
  {
    js_event js;

    read(joy_fd, &js, sizeof(js_event));

    switch (js.type & ~JS_EVENT_INIT)
    {
    case JS_EVENT_AXIS:
      if (static_cast<std::size_t>(js.number) >= joy_axis.size())
      {
        std::cerr << "[error] " << static_cast<std::size_t>(js.number) << std::endl;
        continue;
      }
      joy_axis[static_cast<std::size_t>(js.number)] = js.value;
      break;

    case JS_EVENT_BUTTON:
      if (static_cast<std::size_t>(js.number) >= joy_button.size())
      {
        std::cerr << "[error] " << static_cast<std::size_t>(js.number) << std::endl;
        continue;
      }
      joy_button[static_cast<std::size_t>(js.number)] = js.value;
      break;
    }

    std::cout << "\e[2A\r[debug] axis: ";
    for (const auto& axis : joy_axis)
    {
      std::cout << std::showpos << std::fixed << std::setprecision(3) << static_cast<double>(axis) / -32768 << (&axis != &joy_axis.back() ? ' ' : '\n');
    }

    std::cout << "        button: ";
    for(const auto& button : joy_button)
    {
      std::cout << static_cast<int>(button) << (&button != &joy_button.back() ? ' ' : '\n');
    }

    usleep(100);
  }

  close(joy_fd);
  
  return 0;
}
