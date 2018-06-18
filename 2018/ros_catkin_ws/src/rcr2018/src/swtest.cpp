#include <iostream>
#include <wiringPi.h>

const int START_SW_PIN {25};

int main()
{
  int pin_value {0};

  if (wiringPiSetupGpio() == -1)
  {
     std::cout << "ERROR" << std::endl;
     return -1;
  }

  pinMode(START_SW_PIN, INPUT);

  pin_value = digitalRead(START_SW_PIN);
  while(true)
  {
    std::cout << pin_value << std::endl;
    pin_value = digitalRead(START_SW_PIN);
  }

  return 0;

}
