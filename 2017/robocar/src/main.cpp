#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex> // atanh
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random> // dummy sensor value
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <vector>

#include <robocar/camera/camera.hpp>
#include <robocar/driver/driver.hpp>
#include <robocar/serial/serial.hpp>
#include <robocar/vector/vector.hpp>


const std::unordered_map<std::string,char> sensor_codes {
  {"long_range_0",   5},
  {"long_range_1",   4},
  {"long_range_2",   3},
  {"long_range_3",   2},
  {"long_range_4",   1},
  {"long_range_5",   0},
  {"long_range_6",  -1},
  {"long_range_7",   6},
  {"test_8",  8},
  {"test_9",  9},
  {"short_range_0", 12},
  {"short_range_1", 11},
  {"short_range_2", 10},
  {"test_13", 13},
  {"test_14", 13},
  {"test_15", 13}
};


int main(int argc, char** argv) try
{
  robocar::wiring_serial serial {"/dev/ttyACM0", 9600};

  std::cout << "[debug] wait for serial connection stabilize...\n";
  std::this_thread::sleep_for(std::chrono::seconds(3));

  static constexpr std::size_t width  {640};
  static constexpr std::size_t height {480};
  robocar::camera camera {width, height};

  robocar::differential_driver driver {std::pair<int,int> {35, 38}, std::pair<int,int> {37, 40}};

  auto query = [&](const std::string& name, std::string& dest)
    -> std::string
  {
    if (sensor_codes.find(name) != sensor_codes.end())
    {
      // std::cout << "[debug] name: " << name << std::endl;
      if (name == "long_range_6")
      {
        dest = "45";
        return  dest;
      }

      serial.putchar(static_cast<char>(sensor_codes.at(name)));
      // std::cout << "[debug] putchar: " << static_cast<int>(sensor_codes.at(name)) << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // TODO adjust

      // std::cout << "[debug] avail: " << serial.avail() << std::endl;

      while (serial.avail() > 0)
      {
        dest.push_back(serial.getchar());
      }
      // std::cout << "[debug] dest: " << dest << std::endl;

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

  auto position = [&]()
    -> robocar::vector<double>
  {
    return robocar::vector<double> {0.0, 1.0};
  };

  const auto const_forward_vector = []()
    -> boost::numeric::ublas::vector<double>
  {
    static constexpr std::size_t extent {2};
    boost::numeric::ublas::vector<double> forward {extent};

    forward[0] = 0.0;
    forward[1] = 1.0;

    return {forward};
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

      int sensor_value {short_range_sensor(sensor_value_raw)};
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

  while (true)
  {
    // std::vector<robocar::vector<double>> poles {search()};
    std::vector<robocar::vector<double>> poles {};

    robocar::vector<double> base {poles.empty() == true ? position() : poles.front()};

    base +=  long_range_sensor_array_debug();
    base += short_range_sensor_array_debug();

    std::cout << "[debug] " << base << std::endl;
    driver.write(base, 1.0);
  }

  // {
  //   auto forward = [&]()
  //   {
  //     driver.write(robocar::vector<double> {0.0, 1.0}, static_cast<double>(0.18));
  //   };
  //
  //   auto right_turn = [&]()
  //   {
  //     driver.write(robocar::vector<double> {1.0, 0.0}, static_cast<double>(0.18));
  //   };
  //
  //   auto left_turn = [&]()
  //   {
  //     driver.write(robocar::vector<double> {-1.0, 0.0}, static_cast<double>(0.18));
  //   };
  //
  //   auto stop = [&]()
  //   {
  //     driver.write(robocar::vector<double> {0.0, 0.0}, static_cast<double>(0.18));
  //   };
  //
  //   auto carrot_test = [&]()
  //   {
  //     std::vector<robocar::vector<double>> poles {search()};
  //
  //     robocar::vector<double> base {poles.empty() == true ? robocar::vector<double> {0.0, 0.0} : poles.front()};
  //     base[0] *= 0.3;
  //
  //     std::cout << "[debug] base: " << base << std::endl;
  //     driver.write(base, 0.18);
  //   };
  // }

  // forward();
  // std::this_thread::sleep_for(std::chrono::seconds(3));
  //
  // right_turn();
  // std::this_thread::sleep_for(std::chrono::seconds(3));
  //
  // left_turn();
  // std::this_thread::sleep_for(std::chrono::seconds(3));
  //
  // stop();


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

