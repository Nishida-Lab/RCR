#ifndef INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_
#define INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_


#include <cstdlib>
#include <iostream>
#include <system_error>
#include <utility>

#include <wiringPi.h>
#include <softPwm.h>

#include <robocar/vector/vector.hpp>


namespace robocar {


class differential_driver
{
  std::pair<int,int> pin_;

public:
  differential_driver(int l, int r)
    : pin_ {l, r}
  {
    setup();
    create(pin_.first);
    create(pin_.second);
  }

  differential_driver(const std::pair<int,int>& pair)
    : pin_ {pair}
  {
    setup();
    create(pin_.first);
    create(pin_.second);
  }

public:
  // void write(const std::pair<int,int>& value) noexcept
  // {
  //   softPwmWrite(pin_.first,  value.first);
  //   softPwmWrite(pin_.second, value.second);
  // }

  template <typename T>
  void write(const robocar::vector<T>& v, T tread) const
  {
    static const robocar::vector<T> forward {0.0, 1.0};

    const T  linear_x {v[1] < static_cast<T>(0.0) ? static_cast<T>(0.0) : v[1]};
    const T angular_z {robocar::vector<T>::angle(forward, v) * (v[0] < static_cast<T>(0.0) ? static_cast<T>(1.0) : static_cast<T>(-1.0))};

    const T l {linear_x - tread * static_cast<T>(0.5) * angular_z};
    const T r {linear_x + tread * static_cast<T>(0.5) * angular_z};

    softPwmWrite(pin_.first,  static_cast<int>(l * 100.0));
    softPwmWrite(pin_.second, static_cast<int>(r * 100.0));
  }

private:
  void setup() noexcept
  {
    if (wiringPiSetupPhys() == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiringPiSetupPhys(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  void create(int pin_number) noexcept
  {
    if (softPwmCreate(pin_number, 0, 100) != 0)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] softPwmCreate(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
};


} // robocar


#endif
