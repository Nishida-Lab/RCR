#ifndef INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_
#define INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_


#include <cstdlib>
#include <iostream>
#include <system_error>
#include <utility>

#include <wiringPi.h>
#include <softPwm.h>



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
  void write(const std::pair<int,int>& value) noexcept
  {
    softPwmWrite(pin_.first,  value.first);
    softPwmWrite(pin_.second, value.second);
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
