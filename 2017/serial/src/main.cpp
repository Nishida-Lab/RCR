#include <iostream>
#include <system_error>
#include <utility>

#include <serial/serial.hpp>


int main(int argc, char** argv)
{
  robocar::wiring_serial serial {"/dev/ttyACM0", 115200};

  while (true)
  {
    static char buffer {};

    std::cout << "[debug] input: ";
    std::cin >> buffer;

    serial.putchar(buffer);

    std::cout << "[debug] return: " << serial.getchar() << std::endl;
  }

  return 0;
}


