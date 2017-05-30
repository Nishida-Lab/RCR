#ifndef INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_
#define INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_


#include <cmath>
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
  std::pair<int,int> pwm_pin_;
  std::pair<int,int> dir_pin_;

public:
  differential_driver(const std::pair<int,int>& pwm_pin,
                      const std::pair<int,int>& dir_pin)
    : pwm_pin_ {pwm_pin},
      dir_pin_ {dir_pin}
  {
    setup();

    create(pwm_pin_.first);
    create(pwm_pin_.second);

    pinMode(dir_pin_.first,  OUTPUT);
    pinMode(dir_pin_.second, OUTPUT);
  }

public:
  // void write(const std::pair<int,int>& value) noexcept
  // {
  //   softPwmWrite(pin_.first,  value.first);
  //   softPwmWrite(pin_.second, value.second);
  // }

  template <typename T>
  void debug(const robocar::vector<T>& v, T tread) const
  {
    static const robocar::vector<T> forward {0.0, 1.0};

    const T  linear_x {v[1] < static_cast<T>(0.0) ? static_cast<T>(0.0) : v[1]};
    const T angular_z {robocar::vector<T>::angle(forward, v) * (v[0] < static_cast<T>(0.0) ? static_cast<T>(1.0) : static_cast<T>(-1.0))};

    const T l {linear_x - tread * static_cast<T>(0.5) * angular_z};
    const T r {linear_x + tread * static_cast<T>(0.5) * angular_z};

    std::cout << "[debug] direction L: " << !std::signbit(static_cast<float>(l))  << std::endl
              << "        direction R: " <<  std::signbit(static_cast<float>(r))  << std::endl
              << "        PWM ratio L: " << static_cast<int>(std::abs(l) * 100.0) << std::endl
              << "        PWM ratio R: " << static_cast<int>(std::abs(r) * 100.0) << std::endl;
  }

  template <typename T>
  void write(const robocar::vector<T>& v, T tread) const
  {
    static const robocar::vector<T> forward {0.0, 1.0};

    const T  linear_x {v[1] < static_cast<T>(0.0) ? static_cast<T>(0.0) : v[1]};
    const T angular_z {robocar::vector<T>::angle(forward, v) * (v[0] < static_cast<T>(0.0) ? static_cast<T>(1.0) : static_cast<T>(-1.0))};

    const T l {linear_x - tread * static_cast<T>(0.5) * angular_z};
    const T r {linear_x + tread * static_cast<T>(0.5) * angular_z};

    digitalWrite(dir_pin_.first, !std::signbit(static_cast<float>(l)));
    digitalWrite(dir_pin_.second, std::signbit(static_cast<float>(r)));

    softPwmWrite(pwm_pin_.first,  static_cast<int>(std::abs(l) * 100.0));
    softPwmWrite(pwm_pin_.second, static_cast<int>(std::abs(r) * 100.0));
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
