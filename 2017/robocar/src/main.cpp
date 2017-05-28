#include <iostream>

#include <boost/numeric/ublas/vector.hpp>

#include <robocar/serial/wiring_serial.hpp>


int main(int argc, char** argv)
{
  boost::numeric::ublas::vector<double> direction {};

  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  return 0;
}
