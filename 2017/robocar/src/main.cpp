#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex> // atanh
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include <robocar/camera/camera.hpp>
#include <robocar/driver/driver.hpp>
#include <robocar/sensor/sensor_node.hpp>
#include <robocar/sensor/wiring_serial.hpp> // TODO CLEANUP
#include <robocar/vector/vector.hpp>
#include <robocar/version.hpp>

#include <utilib/unique_fd.hpp>
#include <utilib/runtime_typename.hpp>


int main(int argc, char** argv) try
{
  std::cout << "[debug] project version: " << project_version.data() << " (" << cmake_build_type.data() << ")\n";
  std::cout << "[debug]   boost version: " <<   boost_version.data() << "\n\n";


  robocar::sensor_node<char> sensor {"/dev/ttyACM0", 115200};

  static constexpr std::size_t width  {640};
  static constexpr std::size_t height {480};
  robocar::camera camera {640, 480};

  std::this_thread::sleep_for(std::chrono::seconds(3));

  robocar::differential_driver driver {
    std::pair<int,int> {35, 38}, std::pair<int,int> {37, 40}
  };


  sensor["distance"]["long"]["south_west"].set_code(0);
  sensor["distance"]["long"][      "west"].set_code(1);
  sensor["distance"]["long"]["north_west"].set_code(2);
  sensor["distance"]["long"]["north"     ].set_code(3);
  sensor["distance"]["long"]["north_east"].set_code(4);
  sensor["distance"]["long"][      "east"].set_code(5);
  sensor["distance"]["long"]["south_east"].set_code(6);

  sensor["distance"]["short"]["north_west"].set_code(10);
  sensor["distance"]["short"]["north"     ].set_code(11);
  sensor["distance"]["short"]["north_east"].set_code(12);

  sensor["position"]["x"].set_code(7);
  sensor["position"]["y"].set_code(8);
  sensor["position"]["z"].set_code(9);

  sensor["dummy"]["d"].set_code(13);
  sensor["dummy"]["e"].set_code(14);
  sensor["dummy"]["f"].set_code(15);


  std::vector<std::vector<robocar::vector<double>>> predefined_filed {
    {{ 1.0,  0.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}}
  };

  std::pair<double, double> position {0.0, 0.0};
  double grid_size {0.90}; // [m]
  auto target_vector {predefined_filed[position.second / grid_size][position.first / grid_size]};

  for (double row {0.0}; row < grid_size * predefined_filed.size(); row += 0.1)
  {
    for (double col {0.0}; col < grid_size * predefined_filed[0].size(); col += 0.01)
    {
       predefined_filed[row/grid_size][col/grid_size].normalized();
    }
  }


  auto avoid_vector = [&]()
  {
    robocar::vector<double> result {0.0, 0.0};

    static constexpr double range_min {0.03};
    static constexpr double range_mid {0.45};
    static constexpr double range_max {0.90};

    const std::unordered_map<std::string, robocar::vector<double>> nearest_neighbor {
      {"notrh_west", { 1.0, -1.0}}, {"north"     , { 0.0, -1.0}}, {"north_east", {-1.0, -1.0}},
      {      "west", { 1.0,  1.0}},                               {      "east", {-1.0,  0.0}},
      {"south_west", { 1.0,  1.0}}, {"south"     , { 0.0,  1.0}}, {"south_east", {-1.0,  1.0}}
    };

    for (const auto& psd : sensor["distance"]["long"])
    {
      double buffer;

      if (std::regex_search(psd.first, std::regex {"north"}))
      {
#ifndef NDEBUG
        std::cout << "[debug] regex find \"north\": " << psd.first << std::endl;
#endif
        sensor["distance"]["short"][psd.first] >> buffer;
        buffer = 0.09999 * buffer + 0.4477;

        if (0.18 < buffer)
        {
          *(psd.second) >> buffer;
          buffer = 45.514 * std::pow(buffer, -0.822);
        }
      }
      else
      {
        *(psd.second) >> buffer;
        buffer = 45.514 * std::pow(buffer, -0.822);
      }

      double repulsive_force;

      if (range_min < buffer && buffer < range_mid)
      {
        auto x {(buffer - range_min) / (range_mid - range_min)};
        repulsive_force = -std::atanh(x - 1);
      }
      else if (range_mid <= buffer && buffer < range_max)
      {
        auto x {buffer / range_mid};
        repulsive_force = -std::atanh(x - 1);
      }
      else
      {
        repulsive_force = -std::atanh(range_mid - 1);
      }

      result += nearest_neighbor.at(psd.first) * repulsive_force;
    }

    return result;
  };


  auto search = [&]()
    -> std::vector<robocar::vector<double>>
  {
    std::vector<robocar::vector<double>> poles {};

    for (const auto& p : camera.find())
    {
      int x_pixel {static_cast<int>(p.first)  - static_cast<int>(width / 2)};
      double x_ratio {static_cast<double>(x_pixel) / static_cast<double>(width / 2)};

      poles.emplace_back(x_ratio, std::pow(static_cast<double>(1.0) - std::pow(x_ratio, 2.0), 0.5));
    }

    std::sort(poles.begin(), poles.end(), [&](auto a, auto b) {
      return std::abs(a[0]) < std::abs(b[0]);
    });

    return poles;
  };


  auto carrot_test = [&]()
  {
    std::vector<robocar::vector<double>> poles {search()};

    robocar::vector<double> base {poles.empty() == true ? robocar::vector<double> {0.0, 0.0} : poles.front()};

    std::cout << "[debug] base: " << base << std::endl;
    driver.write(base, 0.18, 0.5);
  };

  auto radio_control = [&]()
  {
    const std::string device {"/dev/input/js0"};
    utilib::unique_fd joy_fd {open(device.c_str(), O_RDONLY)};

    if (joy_fd)
    {
      std::cerr << "[error] failed to open " << device << std::endl;
      std::exit(EXIT_FAILURE);
    }

    int num_of_axis {0}, num_of_buttons {0};

    std::vector<char> joy_button;
    std::vector<int>  joy_axis;

    ioctl(joy_fd, JSIOCGAXES,     &num_of_axis);
    ioctl(joy_fd, JSIOCGBUTTONS,  &num_of_buttons);

    joy_button.resize(num_of_buttons, 0);
    joy_axis.resize(num_of_axis, 0);

    fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode

    while (true)
    {
      js_event js {};

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
  };


  std::cout << "\n";

  for (auto begin = std::chrono::high_resolution_clock::now(),
             last = std::chrono::high_resolution_clock::now();
       std::chrono::duration_cast<std::chrono::seconds>(last - begin) < std::chrono::seconds {10};
       last = std::chrono::high_resolution_clock::now())
  {
    auto poles {search()};

    auto target_vector {
      poles.empty() ? predefined_filed[sensor["position"]["y"].get()/grid_size][sensor["position"]["x"].get()/grid_size] : poles.front().normalized()
    };

#ifndef NDEBUG
    auto  t = std::chrono::duration_cast<std::chrono::seconds>(last - begin);
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last);

    std::cout << "[debug] t: " << t.count() << ", dt: " << dt.count() << "[msec]\n";
#endif
  }

  std::cout << "\n\n";

  return 0;
}

catch (std::logic_error& error)
{
  std::cerr << "[error] logic_error: " << error.what() << std::endl;
  std::exit(EXIT_FAILURE);
}

catch (std::system_error& error)
{
  std::cerr << "[error] code: " << error.code().value() << " - " << error.code().message() << std::endl;
  std::exit(EXIT_FAILURE);
}

catch (...)
{
  std::cerr << "[fatal] An unexpected error occurred. Report the following output to the developer.\n"
            << "        error: " << errno << " - " << std::strerror(errno) << std::endl;

  std::exit(EXIT_FAILURE);
}

