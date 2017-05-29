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

#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <robocar/serial/wiring_serial.hpp>
#include <robocar/camera/camera.hpp>


const std::unordered_map<std::string,std::int8_t> sensor_codes {
  {"long_range_0",  0},
  {"long_range_1",  1},
  {"long_range_2",  2},
  {"long_range_3",  3},
  {"long_range_4",  4},
  {"long_range_5",  5},
  {"long_range_6",  6},
  {"long_range_7",  7},
  {"test_8",  8},
  {"test_9",  9},
  {"short_range_0", 10},
  {"short_range_1", 11},
  {"short_range_2", 12},
  {"test_13", 13}
};


template <typename C, typename T>
std::basic_ostream<C>& operator<<(std::basic_ostream<C>& lhs, const std::pair<T,T>& rhs)
{
  return lhs << "(" << rhs.first << ", " << rhs.second << ")";
}


template <typename T>
auto normalize(const boost::numeric::ublas::vector<T>& v)
  -> boost::numeric::ublas::vector<T>
{
  return {v / boost::numeric::ublas::norm_2(v)};
}


template <typename T>
T dummy_sensor_value(T&& min = static_cast<T>(0.0), T&& max = static_cast<T>(1.0))
{
  static std::default_random_engine engine {std::random_device {}()};
  std::uniform_real_distribution<T> uniform {min, max};
  return uniform(engine);
}


int main(int argc, char** argv) try
{
  // boost::numeric::ublas::vector<double> direction {};

  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  static constexpr std::size_t width  {640};
  static constexpr std::size_t height {480};
  robocar::camera camera {width, height};

  auto query = [&](const std::string& name, std::string&& dest = std::string {}) -> std::string
  {
    if (sensor_codes.find(name) != sensor_codes.end())
    {
      serial.putchar(static_cast<char>(sensor_codes.at(name)));
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // TODO adjust

      // serial.getline(dest);
      dest = std::to_string(dummy_sensor_value(20.0, 180.0)); // dummy data

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

  [[deprecated]] auto camera_test = [&]()
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

  [[deprecated]] auto vector_to_nearest_red_object_debug = [&]()
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
        double vx {static_cast<double>(x) / static_cast<double>(width / 2)};

        objects.emplace_back(vx, std::pow(static_cast<double>(1.0) - std::pow(vx, 2.0), 0.5));
      }

      std::sort(objects.begin(), objects.end(), [&](auto a, auto b) {
        return std::abs(a.first) < std::abs(b.first);
      });

      std::cout << "[debug] ";
      for (auto&& p : objects)
      {
        std::cout << " (" << std::showpos << std::fixed << std::setprecision(3) << p.first
                  << ", " << std::showpos << std::fixed << std::setprecision(3) << p.second << ") ";
      }
      std::putchar('\n');
    }
  };

  auto nearest_pole = [&]()
    -> std::pair<double,double>
  {
    std::vector<std::pair<double,double>> objects {};

    for (const auto& p : camera.find())
    {
      int x {static_cast<int>(p.first)  - static_cast<int>(width/2)};
      double vx {static_cast<double>(x) / static_cast<double>(width / 2)};

      objects.emplace_back(vx, std::pow(static_cast<double>(1.0) - std::pow(vx, 2.0), 0.5));
    }

    std::sort(objects.begin(), objects.end(), [&](auto a, auto b) {
      return std::abs(a.first) < std::abs(b.first);
    });

    return objects.empty() ? std::pair<double,double> {/* dummy value */} : std::pair<double,double> {objects.front()};
  };

  // while (true)
  // {
  //   std::pair<double,double> pole {nearest_pole()};
  //
  //   std::cout << "[debug] " << std::fixed << std::showpos << std::setprecision(3) << pole << std::endl;
  // }

  auto detect_position = [&]()
    -> boost::numeric::ublas::vector<double>
  {
    static constexpr std::size_t extent {2};
    boost::numeric::ublas::vector<double> result {extent};

    result[0] = 0.0;
    result[1] = 1.0;

    return result; // nummy data
  };

  auto add_neighbor = [&]()
  {
    using namespace boost::numeric;
    ublas::vector<double> direction {detect_position()};

    static constexpr std::size_t extent {2};
    std::vector<ublas::vector<double>> neighbor {8, ublas::vector<double> {extent}};

    neighbor[3] <<=  1.0, -1.0;  neighbor[2] <<=  0.0, -1.0;  neighbor[1] <<= -1.0, -1.0;
    neighbor[4] <<=  1.0,  0.0;        /* robocar */          neighbor[0] <<= -1.0,  0.0;
    neighbor[5] <<=  1.0,  1.0;  neighbor[6] <<=  0.0,  1.0;  neighbor[7] <<= -1.0,  1.0;

    for (auto&& v : neighbor)
    {
      v = normalize(v);
      std::cout << "[debug] v_[" << std::noshowpos << &v - &neighbor.front() << "] "
                << std::fixed << std::setprecision(3) << std::showpos << v << std::endl;
    }
  };

  add_neighbor();

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

