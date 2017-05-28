#include <algorithm>
#include <chrono>
#include <cmath>
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

#include <boost/numeric/ublas/vector.hpp>

#include <robocar/serial/wiring_serial.hpp>
#include <robocar/camera/camera.hpp>


const std::unordered_map<std::string,std::int8_t> sensor_codes {
  {"test_0",  0},
  {"test_1",  1},
  {"test_2",  2},
  {"test_3",  3},
  {"test_4",  4},
  {"test_5",  5},
  {"test_6",  6},
  {"test_7",  7},
  {"test_8",  8},
  {"test_9",  9},
  {"short_range_0", 10},
  {"short_range_1", 11},
  {"short_range_2", 12},
  {"test_13", 13}
};


int main(int argc, char** argv) try
{
  boost::numeric::ublas::vector<double> direction {};

  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  auto query = [&](const std::string& name, std::string&& dest = std::string {}) -> std::string
  {
    if (sensor_codes.find(name) != sensor_codes.end())
    {
      serial.putchar(static_cast<char>(sensor_codes.at(name)));
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // TODO adjust
      serial.getline(dest);
      return dest;
    }

    else throw std::logic_error {"std::unordered_map::operator[]() - out of range"};
  };

  constexpr auto acceleration = [&](auto sensor_value) // KXR-94
    -> double
  {
    static constexpr double power_supply {5.00}; // [V]
    static constexpr double AD_conversion_resolution {1024};
    static constexpr double gravitational_acceleration {9.8};

    return (power_supply * (static_cast<double>(sensor_value) - AD_conversion_resolution / 2)) / AD_conversion_resolution * gravitational_acceleration;
  };

  constexpr auto long_range_sensor = [&](auto sensor_value) // GP2Y0A21
    -> double
  {
    return 45.514 * std::pow(static_cast<double>(sensor_value), static_cast<double>(-0.822));
  };

  constexpr auto short_range_sensor = [&](auto sensor_value) // VL6180X
    -> double
  {
    return 0.09999 * static_cast<double>(sensor_value) + 0.4477;
  };

  auto camera_test = [&]()
  {
    static constexpr std::size_t width  {640};
    static constexpr std::size_t height {480};

    robocar::camera camera {width, height};

    while (true)
    {
      std::cout << "[debug] ";
      for (const auto& p : camera.find())
      {
        std::cout << " (" << static_cast<int>(p.first) - static_cast<int>(width/2)
                  << ", " << static_cast<int>(p.second) - static_cast<int>(height/2) << ") ";
      }
      std::putchar('\n');
    }
  };

  auto vector_to_nearest_red_object = [&]()
  {
    static constexpr std::size_t width  {640};
    static constexpr std::size_t height {480};

    robocar::camera camera {width, height};

    while (true)
    {
      std::vector<std::pair<double,double>> objects {};

      for (const auto& p : camera.find())
      {
        int x {static_cast<int>(p.first)  - static_cast<int>(width/2)};
        int y {static_cast<int>(p.second) - static_cast<int>(height/2)};

        objects.emplace_back(static_cast<double>(x) / static_cast<double>(width / 2),
                             static_cast<double>(y) / static_cast<double>(height / 2));
      }

      std::cout << "[debug] ";
      for (auto&& p : objects)
      {
        std::cout << " (" << std::showpos << std::fixed << std::setprecision(3) << p.first
                  << ", " << std::showpos << std::fixed << std::setprecision(3) << p.second << ") ";
      }
      std::putchar('\n');
    }
  };

  vector_to_nearest_red_object();

  return 0;
}

catch (std::logic_error& error)
{
  std::cerr << "[error] " << error.what() << std::endl;
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

