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
  int lpin_, rpin_;

public:
  differential_driver(int lpin, int rpin)
    : lpin_ {lpin},
      rpin_ {rpin}
  {
    setup();
    create(lpin_);
    create(rpin_);
  }

  differential_driver(const std::pair<int,int>& pair)
    : lpin_ {pair.first},
      rpin_ {pair.second}
  {
    setup();
    create(lpin_);
    create(rpin_);
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
