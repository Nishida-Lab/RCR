#include <iostream>
#include <unistd.h>
#include "main.hpp"

using namespace std;

int main()
{
  // --------------------------------
  // WiringPi Test Section
  // --------------------------------
  // wiringPiSetup();
  // pinMode(0, OUTPUT);
  // digitalWrite(0, HIGH); 
  // delay(500);
  // --------------------------------

  // --------------------------------
  // Camera Section 
  // (It has infinity loop)
  // --------------------------------
  // RaspiCamera camera(320,240,0,0,0);
  // while(cvWaitKey(1) != 27){
  //   camera.img = raspiCamCvQueryFrame(camera.cap);
  //   cvShowImage("RasPi Camera", camera.img);
  // }
  // --------------------------------

  Motor motor;  
  motor.Drive(1.23, 0);
  sleep(5);
  motor.Drive(0,0);
  motor.Drive(-1.23, 0);
  sleep(5);
  motor.Drive(0,0);

  return 0 ;
}
