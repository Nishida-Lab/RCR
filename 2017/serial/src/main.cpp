#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <wiringSerial.h>


int main(int argc, char** argv)
{
  auto fd = serialOpen("/dev/ttyACM0", 9600);

  if (fd == -1)
  {
    std::error_code error {errno, std::generic_category()};
    std::cerr << "[error] wiringSerial(3) - " << error.message() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  while (true)
  {
    static char buffer {};

    std::cout << "[debug] input: ";
    std::cin >> buffer;

    serialPutchar(fd, buffer);

    std::cout << "[debug] return: " << serialGetchar(fd) << std::endl;
  }

  return 0;
}


