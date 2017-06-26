#ifndef INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_
#define INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_


#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include <wiringSerial.h>


namespace robocar {


template <typename C>
class wiring_serial
{
  static std::size_t reference_count_;
  C code_;

public:
  int fd;

  wiring_serial(const std::basic_string<C>& device, int baudrate)
    : fd {serialOpen(device.c_str(), baudrate)}
  {
    if (fd == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiring_serial::wiring_serial(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    ++reference_count_;

#ifndef NDEBUG
    std::cout << "[debug] robocar::wiring_serial::wiring_serial(" << __LINE__ << ") - reference count: "
              << reference_count_ << std::endl;
#endif
  }

  explicit wiring_serial(const robocar::wiring_serial<C>& parent)
    : fd {parent.fd}
  {
    ++reference_count_;

#ifndef NDEBUG
    std::cout << "[debug] robocar::wiring_serial::wiring_serial(" << __LINE__ << ") - reference count: "
              << reference_count_ << std::endl;
#endif
  }

  ~wiring_serial()
  {
#ifndef NDEBUG
    std::cout << "[debug] robocar::wiring_serial::~wiring_serial() - reference count: "
              << reference_count_ << std::endl;
#endif

    if (!--reference_count_)
    {
#ifndef NDEBUG
      std::cout << "[debug] robocar::wiring_serial::~wiring_serial() - serial close\n";
      serialClose(fd);
#endif
    }
  }

public:
  auto& set_code(const C& rhs) // XXX UGLY CODE
  {
    code_ = rhs;
    return *this;
  }

  // template <typename C>
  // auto& operator>>(const robocar::wiring_serial<C>& lhs, std::basic_string<C>& rhs)
  // {
  //   std::cout << "[debug] putchar: " << static_cast<int>(code_) << std::endl;
  //   return lhs;
  // }

  auto& operator>>(std::basic_string<C>& rhs)
  {
    std::cout << "[debug] putchar: " << static_cast<int>(code_) << std::endl;
    putchar(code_);

    while (!avail())
    {
      std::cout << "[debug] wait for avail...\n";
      std::this_thread::sleep_for(std::chrono::milliseconds {1});
    }

    getline(rhs);

    return *this;
  }

public:
  template <typename... Ts>
  void putchar(Ts&&... args)
  {
    serialPutchar(fd, std::forward<Ts>(args)...);
  }

  // template <typename... Ts>
  // void puts(Ts&&... args)
  // {
  //   serialPuts(fd, std::forward<Ts>(args)...);
  // }

  // template <typename... Ts>
  // void printf(Ts&&... args)
  // {
  //   serialPrintf(fd, std::forward<Ts>(args)...);
  // }

  std::size_t avail()
  {
    int size {serialDataAvail(fd)};

    if (size == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiring_serial::avail(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    return static_cast<std::size_t>(size);
  }

  C getchar()
  {
    return static_cast<C>(serialGetchar(fd));
  }

  void getline(std::basic_string<C>& dest, C delim = '\n')
  {
    for (C buffer {}; (buffer = getchar()) != delim; dest.push_back(buffer));
  }

  void flush()
  {
    serialFlush(fd);
  }
};


} // namespace robocar


template <typename C>
std::size_t robocar::wiring_serial<C>::reference_count_;


#endif

