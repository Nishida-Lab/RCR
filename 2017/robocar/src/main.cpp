#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>

#include <boost/numeric/ublas/vector.hpp>

#include <robocar/serial/wiring_serial.hpp>


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

  auto query = [&](const std::string& name, std::string&& dest = std::string {})
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

