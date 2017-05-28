#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <wiringSerial.h>

#include <serial/wiring_serial.hpp>


// namespace robocar {
//
//
// class serial
// {
//   decltype(serialOpen(std::declval<char*>{},std::declval<int>{})) fd_;
//
// public:
//   template <typename... Ts>
//   serial(Ts&&... args)
//     : fd_ {serialOpen(std::forward<Ts>(args)...)}
//   {
//     if (fd_ == -1)
//     {
//       std::error_code error {errno, std::generic_category()};
//       std::cerr << "[error] wiringSerial(3) - " << error.message() << std::endl;
//       std::exit(EXIT_FAILURE);
//     }
//   }
//
//   ~serial()
//   {
//     serialClose(fd_);
//   }
//
//   template <typename... Ts>
//   void putchar(Ts&&... args)
//   {
//     serialPutchar(fd_, std::forward<Ts>(args)...);
//   }
//
//   template <typename... Ts>
//   void puts(Ts&&... args)
//   {
//     serialPuts(fd_, std::forward<Ts>(args)...);
//   }
//
//   template <typename... Ts>
//   void printf(Ts&&... args)
//   {
//     serialPrintf(fd_, std::forward<Ts>(args)...);
//   }
//
//   template <typename T = char>
//   T getchar()
//   {
//     return static_cast<T>(serialGetchar(fd_));
//   }
//
//   void flush()
//   {
//     serialFlush(fd_);
//   }
// };
//
//
// } // namespace robocar


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


