#ifndef INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_
#define INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_


#include <iostream>
#include <string>
#include <system_error>
#include <utility>

#include <wiringSerial.h>


namespace robocar {


template <typename C>
class wiring_serial
{
  int fd_;

public:
  wiring_serial(const std::basic_string<C>& device, int baudrate)
    : fd_ {serialOpen(device.c_str(), baudrate)}
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

  C getchar()
  {
    return static_cast<C>(serialGetchar(fd_));
  }

  void getline(std::basic_string<C>& dest, C delim = '\n')
  {
    for (C buffer {}; (buffer = getchar()) != delim; dest.push_back(buffer));
  }

  void flush()
  {
    serialFlush(fd_);
  }
};


} // namespace robocar


#endif

