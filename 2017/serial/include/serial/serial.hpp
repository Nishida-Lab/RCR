#ifndef INCLUDED_ROBOCAR_SERIAL_SERIAL_HPP_
#define INCLUDED_ROBOCAR_SERIAL_SERIAL_HPP_


#include <iostream>
#include <string>
#include <system_error>
#include <utility>

#include <wiringSerial.h>


namespace robocar {


class wiring_serial
{
  decltype(serialOpen(std::declval<char*>{},std::declval<int>{})) fd_;

public:
  template <typename... Ts>
  wiring_serial(Ts&&... args)
    : fd_ {serialOpen(std::forward<Ts>(args)...)}
  {
    if (fd_ == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiring_serial::wiring_serial(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  ~wiring_serial()
  {
    serialClose(fd_);
  }

  template <typename... Ts>
  void putchar(Ts&&... args)
  {
    serialPutchar(fd_, std::forward<Ts>(args)...);
  }

  template <typename... Ts>
  void puts(Ts&&... args)
  {
    serialPuts(fd_, std::forward<Ts>(args)...);
  }

  template <typename... Ts>
  void printf(Ts&&... args)
  {
    serialPrintf(fd_, std::forward<Ts>(args)...);
  }

  std::size_t avail()
  {
    int size {serialDataAvail(fd_)};

    if (size == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiring_serial::avail(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    else return static_cast<std::size_t>(size);
  }

  template <typename C = char>
  C getchar()
  {
    return static_cast<C>(serialGetchar(fd_));
  }

  template <typename C = char>
  void getline(std::basic_string<C>& dest, C delim = '\n')
  {
    for (C buffer {}; (buffer = getchar<C>()) != delim; dest.push_back(buffer));
  }

  void flush()
  {
    serialFlush(fd_);
  }
};


} // namespace robocar


#endif
