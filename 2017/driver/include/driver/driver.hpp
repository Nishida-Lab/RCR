#ifndef INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_
#define INCLUDED_ROBOCAR_DRIVER_DRIVER_HPP_


#include <cstdlib>
#include <iostream>
#include <system_error>

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
    if (wiringPiSetupPhys() == -1)
    {
      std::error_code error {errno, std::generic_category()};
      std::cerr << "[error] wiringPiSetupPhys(3) - " << error.message() << std::endl;
      std::exit(EXIT_FAILURE);
    }

    create(lpin_);
    create(rpin_);
  }


private:
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
