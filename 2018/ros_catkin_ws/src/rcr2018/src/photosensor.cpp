#include <iostream>
#include <wiringPi.h>
#include <unistd.h>

#define GPIO4 4

int count = 0;

int sensor_charge()
    {
      pinMode(4,OUTPUT);

      digitalWrite(4,1);

      usleep(10);

      pinMode(4,INPUT);
    }

int main(){

  if(wiringPiSetupGpio() == -1)
    {
      std::cout << "ERROR:Can't setup GPIO." << std::endl;
      return -1;
    }

  pullUpDnControl(4,PUD_UP);

  while (1) {


  sensor_charge();


    while (digitalRead(GPIO4) == 1)
    {
      printf("Black\n");
    }

    while (digitalRead(GPIO4) == 0)
    {
      printf("White\n");
    }

}
return 0;
}
