#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex> // atanh
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
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
#include <robocar/sensor/wiring_serial.hpp>
#include <robocar/vector/vector.hpp>
#include <robocar/version.hpp>

#include <utilib/unique_fd.hpp>
#include <utilib/runtime_typename.hpp>


const std::unordered_map<std::string, char> sensor_codes {
  {"long_range_0",   5},
  {"long_range_1",   4},
  {"long_range_2",   3},
  {"long_range_3",   2},
  {"long_range_4",   1},
  {"long_range_5",   0},
  {"long_range_6",  -1},
  {"long_range_7",   6},
  {"accel_x",  7},
  {"accel_y",  8},
  {"accel_y",  9},
  {"short_range_0", 12},
  {"short_range_1", 11},
  {"short_range_2", 10},
  {"gyro_x", 13},
  {"gyro_y", 14},
  {"gyro_z", 15}
};


int main(int argc, char** argv) try
{
  std::cout << "[debug] project version: " << project_version.data() << " (" << cmake_build_type.data() << ")\n";
  std::cout << "[debug]   boost version: " <<   boost_version.data() << "\n\n";

  robocar::sensor_node<char> sensor {"/dev/ttyACM0", 9600};

  sensor["distance"]["long"]["south_west"].set_code(0);
  sensor["distance"]["long"][      "west"].set_code(1);
  sensor["distance"]["long"]["north_west"].set_code(2);
  sensor["distance"]["long"]["north"     ].set_code(3);
  sensor["distance"]["long"]["north_east"].set_code(4);
  sensor["distance"]["long"][      "east"].set_code(5);
  sensor["distance"]["long"]["south_east"].set_code(6);

  sensor["dummy"]["a"].set_code(7);
  sensor["dummy"]["b"].set_code(8);
  sensor["dummy"]["c"].set_code(9);

  sensor["distance"]["short"]["north_west"].set_code(10);
  sensor["distance"]["short"]["north"     ].set_code(11);
  sensor["distance"]["short"]["north_east"].set_code(12);

  sensor["dummy"]["d"].set_code(13);
  sensor["dummy"]["e"].set_code(14);
  sensor["dummy"]["f"].set_code(15);

  std::this_thread::sleep_for(std::chrono::seconds(3));


#ifdef NDEBUG
  robocar::wiring_serial serial {"/dev/ttyACM0", 9600};

  std::cout << "[debug] wait for serial connection stabilize...\n";
  std::this_thread::sleep_for(std::chrono::seconds(3));

  static constexpr std::size_t width  {640};
  static constexpr std::size_t height {480};
  robocar::camera camera {width, height};

  robocar::differential_driver driver {
    std::pair<int,int> {35, 38}, std::pair<int,int> {37, 40}
  };

  auto query = [&](const std::string& name, std::string& dest) // TODO remake
    -> std::string
  {
    if (sensor_codes.find(name) != sensor_codes.end())
    {
      if (name == "long_range_6")
      {
        dest = "45";
        return  dest;
      }

      serial.putchar(static_cast<char>(sensor_codes.at(name)));
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // TODO adjust

      while (serial.avail() > 0)
      {
        dest.push_back(serial.getchar());
      }

      return dest;
    }

    else throw std::logic_error {"std::unordered_map::operator[]() - out of range"};
  };

  constexpr auto long_range_sensor = [&](auto sensor_value) // GP2Y0A21
    -> double
  {
    double tmp {sensor_value * 5 / 1024};
    return 45.514 * std::pow(static_cast<double>(tmp), static_cast<double>(-0.822));
  };

  constexpr auto short_range_sensor = [&](auto sensor_value) // VL6180X
    -> double
  {
    return 0.09999 * static_cast<double>(sensor_value) + 0.4477;
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

  auto long_range_sensor_array_debug = [&]()
    -> boost::numeric::ublas::vector<double>
  {
    boost::numeric::ublas::vector<double> direction {robocar::vector<double> {0.0, 0.0}};

    static constexpr std::size_t extent {2};
    std::vector<boost::numeric::ublas::vector<double>> neighbor {8, boost::numeric::ublas::vector<double> {extent}};

    neighbor[3] <<=  1.0, -1.0;  neighbor[2] <<=  0.0, -1.0;  neighbor[1] <<= -1.0, -1.0;
    neighbor[4] <<=  1.0,  0.0;        /* robocar */          neighbor[0] <<= -1.0,  0.0;
    neighbor[5] <<=  1.0,  1.0;  neighbor[6] <<=  0.0,  1.0;  neighbor[7] <<= -1.0,  1.0;

    // for (auto&& v : neighbor)
    // {
    //   v = robocar::vector<double>::normalize(v);
    //   std::cout << "[debug] neighbor[" << std::noshowpos << &v - &neighbor.front() << "] "
    //             << std::fixed << std::setprecision(3) << std::showpos << v << std::endl;
    // }

    static constexpr std::size_t desired_distance {45};

    for (const auto& v : neighbor)
    {
      int index {static_cast<int>(&v - &neighbor.front())};
      std::cout << "[debug] index: " << index << std::endl;

      std::string sensor_name {"long_range_" + std::to_string(index)};
      std::cout << "        sensor name: " << sensor_name << std::endl;

      std::string sensor_value_str {};
      query(sensor_name, sensor_value_str);
      std::cout << "        sensor value str: " << sensor_value_str << std::endl;

      int sensor_value_raw {std::stoi(sensor_value_str)};
      std::cout << "        sensor value raw: " << sensor_value_raw << std::endl;

      double sensor_value {long_range_sensor(sensor_value_raw)};
      std::cout << "        sensor value: " << sensor_value << std::endl;

      double range_max {desired_distance * 2};

      if (sensor_value > range_max)
      {
        sensor_value = desired_distance;
      }

      double distance {static_cast<double>(sensor_value) - static_cast<double>(desired_distance)};
      std::cout << "        distance from desired position: " << distance << std::endl;

      double normalized_distance {distance / static_cast<double>(desired_distance)};
      std::cout << "        normalized distance: " << normalized_distance << std::endl;

      double arctanh {-std::atanh(normalized_distance)};
      std::cout << "        arctanh: " << arctanh << std::endl;

      boost::numeric::ublas::vector<double> repulsive_force {v * arctanh};
      std::cout << "        repulsive force: " << repulsive_force << std::endl;

      direction += repulsive_force;
      std::cout << "        direction: " << direction << std::endl;
    }

    return direction;
  };

  auto short_range_sensor_array_debug = [&]()
    -> boost::numeric::ublas::vector<double>
  {
    boost::numeric::ublas::vector<double> direction {robocar::vector<double> {0.0, 0.0}};

    static constexpr std::size_t extent {2};
    std::vector<boost::numeric::ublas::vector<double>> neighbor {3, boost::numeric::ublas::vector<double> {extent}};

    neighbor[2] <<=  1.0, -1.0;  neighbor[1] <<=  0.0, -1.0;  neighbor[0] <<= -1.0, -1.0;

    // for (auto&& v : neighbor)
    // {
    //   v = robocar::vector<double>::normalize(v);
    //   std::cout << "[debug] neighbor[" << std::noshowpos << &v - &neighbor.front() << "] "
    //             << std::fixed << std::setprecision(3) << std::showpos << v << std::endl;
    // }

    static constexpr std::size_t desired_distance {3}; // [cm]
    static constexpr std::size_t range_max {20};

    for (const auto& v : neighbor)
    {
      int index {static_cast<int>(&v - &neighbor.front())};
      std::cout << "[debug] index: " << index << std::endl;

      std::string sensor_name {"short_range_" + std::to_string(index)};
      std::cout << "        sensor name: " << sensor_name << std::endl;

      std::string sensor_value_str {};
      query(sensor_name, sensor_value_str);
      std::cout << "        sensor value str: " << sensor_value_str << std::endl;

      int sensor_value_raw {std::stoi(sensor_value_str)};
      std::cout << "        sensor value raw: " << sensor_value_raw << std::endl;

      auto sensor_value {short_range_sensor(sensor_value_raw)};
      std::cout << "        sensor value: " << sensor_value << std::endl;

      double arctanh {};

      if (sensor_value < static_cast<int>(desired_distance))
      {
        std::cout << "[debug] break point " << __LINE__ <<std::endl;
        double numerator   {static_cast<double>(desired_distance) - static_cast<double>(sensor_value)};
        double denominator {static_cast<double>(desired_distance)};

        arctanh = std::atanh(numerator / denominator);
      }

      else
      {
        std::cout << "[debug] break point " << __LINE__ <<std::endl;
        if (sensor_value > static_cast<int>(range_max))
        {
          std::cout << "[debug] break point " << __LINE__ <<std::endl;
          // sensor_value = static_cast<int>(range_max);
          sensor_value = static_cast<int>(desired_distance);
        }

        double numerator   {static_cast<double>(sensor_value) - static_cast<double>(desired_distance)};
        double denominator {static_cast<double>(range_max) - static_cast<double>(desired_distance)};

        arctanh = -std::atanh(numerator / denominator);
      }

      std::cout << "        arctanh: " << arctanh << std::endl;

      boost::numeric::ublas::vector<double> repulsive_force {v * arctanh};
      std::cout << "        repulsive force: " << repulsive_force << std::endl;

      direction += repulsive_force;
      std::cout << "        direction: " << direction << std::endl;
    }

    return direction;
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
#endif

  for (auto begin = std::chrono::high_resolution_clock::now(),
             last = std::chrono::high_resolution_clock::now();
       std::chrono::duration_cast<std::chrono::seconds>(last - begin) < std::chrono::seconds {10};
       last = std::chrono::high_resolution_clock::now())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds {10});

#ifndef NDEBUG
    auto  t = std::chrono::duration_cast<std::chrono::seconds>(last - begin);
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - last);

    std::cout << "\r\e[K[debug] t: " << t.count() << ", dt: " << dt.count() << "[microsec]" << std::flush;
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

