#include <chrono>
#include <complex> // atanh
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

#include <robocar/camera/camera.hpp>
#include <robocar/driver/driver.hpp>
#include <robocar/driver/radio_controler.hpp>
#include <robocar/sensor/sensor_node.hpp>
#include <robocar/sensor/wiring_serial.hpp>
#include <robocar/vector/vector.hpp>
#include <robocar/version.hpp>


int main(int argc, char** argv) try
{
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;

  using std::chrono::seconds;
  using std::chrono::milliseconds;
  using std::chrono::microseconds;

#ifndef NDEBUG
  std::cout << "[debug] project version: " << project_version.data() << " (" << cmake_build_type.data() << ")\n";
  std::cout << "[debug]   boost version: " <<   boost_version.data() << "\n\n";
#endif


  robocar::sensor_node<char> sensor {"/dev/ttyACM0", 115200};

  robocar::camera camera {640, 480};

  robocar::differential_driver driver {
    std::pair<int,int> {35, 38}, std::pair<int,int> {37, 40}
  };


  sensor["distance"]["long"]["south_west"].set(0);
  sensor["distance"]["long"][      "west"].set(1);
  sensor["distance"]["long"]["north_west"].set(2);
  sensor["distance"]["long"]["north"     ].set(3);
  sensor["distance"]["long"]["north_east"].set(4);
  sensor["distance"]["long"][      "east"].set(5);
  sensor["distance"]["long"]["south_east"].set(6);

  sensor["distance"]["short"]["north_west"].set(10);
  sensor["distance"]["short"]["north"     ].set(11);
  sensor["distance"]["short"]["north_east"].set(12);

  sensor["accel"]["x"].set(7);
  sensor["accel"]["y"].set(8);
  sensor["accel"]["z"].set(9);

  sensor["angle"]["x"].set(13);
  sensor["angle"]["y"].set(14);
  sensor["angle"]["z"].set(15);


  std::vector<std::vector<robocar::vector<double>>> predefined_field {
    {{ 1.0,  0.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, { 0.0, -1.0}, {-1.0, -1.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}},
    {{ 0.0,  1.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}, {-1.0,  0.0}}
  };

  for (auto&& row : predefined_field) { for (auto&& v : row) { v = v.normalized(); } }


  auto avoid_vector = [&](double range_min, double range_mid, double range_max)
  {
    robocar::vector<double> result {0.0, 0.0};

    const std::unordered_map<std::string, robocar::vector<double>> nearest_neighbor {
      {"north_west", { 0.707, -0.707}}, {"north", { 0.000, -1.000}}, {"north_east", {-0.707, -0.707}},
      {      "west", { 1.000,  0.000}},                              {      "east", {-1.000,  0.000}},
      {"south_west", { 0.707,  0.707}}, {"south", { 0.000,  1.000}}, {"south_east", {-0.707,  0.707}}
    };

    for (const auto& psd : sensor["distance"]["long"])
    {
      double buffer;

      if (std::regex_search(psd.first, std::regex {"north"}))
      {
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
        double x {(buffer - range_min) / (range_mid - range_min)};
        repulsive_force = -std::atanh(x - 1.0);
      }
      else if (range_mid <= buffer && buffer < range_max)
      {
        double x {buffer / range_mid};
        repulsive_force = -std::atanh(x - 1.0);
      }
      else
      {
        repulsive_force = -std::atanh(range_mid / range_mid - 1);
      }

      // std::cout << "[debug] " << psd.first << ", " << repulsive_force << std::endl;
      result += nearest_neighbor.at(psd.first) * -repulsive_force;
    }

    return result;
  };


  std::cout << "[debug] please wait";
  for (std::size_t count {0}; count < 3; ++count)
  {
    std::cout << "." << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds {1});
  }
  std::cout << std::endl;


  for (auto begin = high_resolution_clock::now(), last = high_resolution_clock::now();
       duration_cast<seconds>(last - begin) < seconds {30};
       last = high_resolution_clock::now())
  {
    // auto poles {camera.search()};

    // auto target_vector {
    //   poles.empty() ? predefined_filed[sensor["position"]["y"].get()/grid_size][sensor["position"]["x"].get()/grid_size] : poles.front().normalized()
    // };

    robocar::vector<double> direction {avoid_vector(0.03, 0.45, 0.90)};
    std::cout << "\r\e[K[debug] " << direction.normalized();
    driver.write(direction, 0.18, 0.5);

#ifndef NDEBUG
    // auto  t = duration_cast<seconds>(last - begin);
    // auto dt = duration_cast<microseconds>(high_resolution_clock::now() - last);
    //
    // std::cout << std::fixed << std::setw(2)
    //           << ", t: " << t.count()
    //           << ", dt: " << dt.count() << "[msec]" << std::flush;
#endif
  }

  driver.write(robocar::vector<double> {0.0, 0.0}, 0.18, 0.3);


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

