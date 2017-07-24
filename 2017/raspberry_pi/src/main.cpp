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
#include <robocar/chrono/for_duration.hpp>
#include <robocar/driver/driver.hpp>
#include <robocar/driver/radio_controller.hpp>
#include <robocar/graph/labeled_tree.hpp>
#include <robocar/sensor/wiring_serial.hpp>
#include <robocar/string/runtime_typename.hpp>
#include <robocar/utility/renamed_pair.hpp>
#include <robocar/vector/vector.hpp>
#include <robocar/version.hpp>


static const robocar::vector<double>
  north_west {-0.707,  0.707}, north { 0.000,  1.000}, north_east { 0.707,  0.707},
        west {-1.000,  0.000},                               east { 1.000,  0.000},
  south_west {-0.707, -0.707}, south { 0.000, -1.000}, south_east { 0.707, -0.707};


static const std::vector<std::vector<robocar::vector<double>>> predefined_field {
  {      east,       east,       east,       east,       east, south     },
  {north     , south     , south     , south     , south     , south_west},
  {north     , south     , south     , south     , south_west,       west},
  {north     , south     , south     , south_west,       west,       west},
  {north     , south     , south_west,       west,       west,       west},
  {north     ,       west,       west,       west,       west,       west}
};


int main(int argc, char** argv) try
{
  robocar::graph::labeled_tree<
    std::string, robocar::wiring_serial<char>
  > sensor {"/dev/ttyACM0", 9600};

  robocar::camera camera {640, 480};
  robocar::differential_driver driver {{35, 38}, {37, 40}};


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

  sensor["position"]["x"].set(7);
  sensor["position"]["y"].set(8);
  sensor["position"]["z"].set(9);

  sensor["angle"]["x"].set(13);
  sensor["angle"]["y"].set(14);
  sensor["angle"]["z"].set(15);


  auto distract_vector = [&](double range_min, double range_mid, double range_max)
  { // TODO OPTIMIZE
    robocar::vector<double> result {0.0, 0.0};

    static const std::unordered_map<std::string, robocar::vector<double>> nearest_neighbor {
      {"north_west", { 0.707, -0.707}}, {"north", { 0.000, -1.000}}, {"north_east", {-0.707, -0.707}},
      {      "west", { 1.000,  0.000}},                              {      "east", {-1.000,  0.000}},
      {"south_west", { 0.707,  0.707}}, {"south", { 0.000,  1.000}}, {"south_east", {-0.707,  0.707}}
    };

    for (const auto& psd : sensor["distance"]["long"])
    {
      double buffer;
      static const std::regex north_regex {"^north.*$"};

      if (std::regex_match(psd.first, north_regex))
      {
        sensor["distance"]["short"][psd.first] >> buffer;
        buffer = (0.09999 * buffer + 0.4477);

        if (0.18 < buffer)
        {
          *(psd.second) >> buffer;
          buffer = (45.514 * std::pow(buffer, -0.822));
        }
      }
      else
      {
        *(psd.second) >> buffer;
        buffer = (45.514 * std::pow(buffer, -0.822));
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

      // std::cout << "[debug] " << psd.first << ":\t" << repulsive_force << std::endl;
      result += nearest_neighbor.at(psd.first) * repulsive_force;
    }

    return result;
  };


  auto attract_vector = [&]()
  { // XXX UGLY CODE!!!
    double current_angle_in_world_coordinate {std::stod(sensor["angle"]["z"].get())};

    robocar::vector<double> current_direction_in_world_coordinate {
      std::cos(robocar::vector<double>::degree_to_radian(current_angle_in_world_coordinate + 90)),
      std::sin(robocar::vector<double>::degree_to_radian(current_angle_in_world_coordinate + 90)),
    };


    robocar::vector<double> target_direction_in_world_coordinate {
      predefined_field[std::max(0.0, std::min(static_cast<double>(predefined_field.size()), std::stod(sensor["position"]["y"].get()) / 0.90))]
                      [std::max(0.0, std::min(static_cast<double>(predefined_field.size()), std::stod(sensor["position"]["x"].get()) / 0.90))]
    };

    double target_angle_in_local_coordinate {robocar::vector<double>::angle(
      current_direction_in_world_coordinate,
       target_direction_in_world_coordinate
    )};

    return robocar::vector<double> {
      std::cos(robocar::vector<double>::degree_to_radian(target_angle_in_local_coordinate)),
      std::sin(robocar::vector<double>::degree_to_radian(target_angle_in_local_coordinate)),
    };
  };


  robocar::chrono::for_duration(std::chrono::seconds {5}, [](auto&& elapsed, auto&& duration)
  {
    std::cout << "\r\e[K[debug] please wait for " << duration.count() - elapsed.count() << " sec" << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds {1});
  });


  robocar::chrono::for_duration(std::chrono::milliseconds {60 * 1000}, [&](auto& elapsed, auto& duration)
  {
    // static std::pair<robocar::vector<double>, decltype(elapsed)> feedback_vector {
    //   {0.0, 0.0}, elapsed
    // };
    //
    // static constexpr double feedback_gein {1.0};

    const robocar::vector<double> distractor {distract_vector(0.03, 0.45, 0.90).normalized()};
    const robocar::vector<double>  attractor {0.0, 0.0};

    const auto toward_fire {
      camera.capture(camera.untested_filter, camera.size.width).normalized()
    };

    robocar::vector<double> direction {
      distractor + (toward_fire[0] == 0.0 && toward_fire[1] == 0.0 ? attractor : toward_fire)
      // + feedback_vector.first * std::exp(-feedback_gein * (elapsed - feedback_vector.second).count() * 0.001)
    };

    driver.write(direction.normalized(), 0.18, 0.5);
    // feedback_vector = std::make_pair(direction, elapsed);

#ifndef NDEBUG
    // std::cout << std::fixed << std::showpos << std::setprecision(3)
    //           << "\r\e[K[debug] distractor: " << distractor << "\n"
    //           << "\r\e[K         attractor: " <<  attractor << " (";
    //
    // if (!poles.empty()) { std::cout << poles.front().normalized() << ")\n"; }
    // else { std::cout << "empty)\n"; }
    //
    // std::cout << "\r\e[K         direction: " <<  direction << " (" << history.size() << ")\e[3A" << std::flush;
#endif
  });

  driver.write(robocar::vector<double> {0.0, 0.0}, 0.18, 0.0);

  return 0;
}

catch (std::system_error& error)
{
  std::cerr << "[error] system error occurred\n"
            << "        code: " << error.code().value() << " - " << error.code().message() << std::endl;
  std::exit(EXIT_FAILURE);
}

catch (std::exception& error)
{
  std::cerr << "[error] caught standard exception - " << error.what() << std::endl;
  std::exit(EXIT_FAILURE);
}

catch (...)
{
  std::cerr << "[fatal] unexpected error occurred. report this to the developer.\n"
            << "        errno: " << errno << " - " << std::strerror(errno) << std::endl;
  std::exit(EXIT_FAILURE);
}

