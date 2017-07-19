#ifndef INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_
#define INCLUDED_ROBOCAR_SENSOR_WIRING_SERIAL_HPP_


#include <chrono>
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
  }

  explicit wiring_serial(const robocar::wiring_serial<C>& parent)
    : fd {parent.fd}
  {
    ++reference_count_;
  }

  ~wiring_serial()
  {
    if (!--reference_count_)
    {
      serialClose(fd);
    }
  }

public:
  auto& set(const C& rhs) // XXX ABSTRUCTION
  {
    code_ = rhs;
    return *this;
  }

  std::basic_string<C>& get() // TODO IMPLEMENT
  {
    write(fd, &code_, 1);
    flush();

    std::this_thread::sleep_for(std::chrono::milliseconds {1});

    static std::basic_string<C> result {};
    result.clear();

    while (true)
    {
      for (auto begin = std::chrono::high_resolution_clock::now(),
                 last = std::chrono::high_resolution_clock::now();
           serialDataAvail(fd) == 0;
           last = std::chrono::high_resolution_clock::now())
      {
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(last - begin);
        std::cout << "\r\e[K[debug] waiting: " << time.count() << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds {1});
      }

      C buffer = static_cast<C>(getchar());

      if (buffer != '\n')
      {
        result.push_back(buffer);
      }
      else break;
    }

    flush();

    std::cout << result << std::flush;

    return result;
  }

  auto& operator>>(std::basic_string<C>& rhs)
  {
    rhs = get();
    return *this;
  }

  auto& operator>>(double& rhs)
  {
    rhs = std::stod(get());
    return *this;
  }

public:
  template <typename... Ts>
  [[deprecated]] void putchar(Ts&&... args)
  {
    serialPutchar(fd, std::forward<Ts>(args)...);
  }

  auto getchar()
  {
    return serialGetchar(fd);
  }

  [[deprecated]] std::size_t avail()
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

  void flush()
  {
    serialFlush(fd);
  }
};


} // namespace robocar


template <typename C>
std::size_t robocar::wiring_serial<C>::reference_count_;


#endif

