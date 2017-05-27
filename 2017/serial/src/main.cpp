#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <wiringSerial.h>


namespace robocar {


class serial
{
  decltype(serialOpen(std::declval<char*>{},std::declval<int>{})) fd_;

public:
  template <typename... Ts>
  serial(Ts&&... args)
    : fd_ {serialOpen(std::forward<Ts>(args)...)}
  {}

  template <typename... Ts>
  void putchar(Ts&&... args)
  {
    serialPutchar(std::forward<Ts>(args)...);
  }

  template <typename... Ts>
  void puts(Ts&&... args)
  {
    serialPuts(std::forward<Ts>(args)...);
  }
};


} // namespace robocar


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


