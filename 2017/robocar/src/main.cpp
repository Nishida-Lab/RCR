#include <cstdint>
#include <iostream>
#include <string>
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


int main(int argc, char** argv)
{
  boost::numeric::ublas::vector<double> direction {};

  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  return 0;
}
