#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>

#include <boost/numeric/ublas/vector.hpp>

#include <robocar/serial/wiring_serial.hpp>


const std::unordered_map<std::string,std::int8_t> sensor_codes {
  // {"",  0},
  // {"",  1},
  // {"",  2},
  // {"",  3},
  // {"",  4},
  // {"",  5},
  // {"",  6},
  // {"",  7},
  // {"",  8},
  // {"",  9},
  {"short_range_0", 10},
  {"short_range_1", 11},
  {"short_range_2", 12},
  // {"", 13}
};


int main(int argc, char** argv) try
{
  boost::numeric::ublas::vector<double> direction {};

  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  auto query = [&](const std::string& name)
  {
    if (sensor_codes.find(name) != sensor_codes.end())
    {
      serial.putchar(static_cast<char>(sensor_codes[name]));
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // TODO adjust
    }

    else
    {
      throw std::logic_error {"std::unordered_map::operator[]() - out of range"};
    }
  };

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

