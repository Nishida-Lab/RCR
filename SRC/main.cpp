#include <wiringPi.h>
#include <iostream>
#include "main.hpp"

using namespace std;

int main()
{
  // --------------------------------
  // WiringPi Test Section
  // --------------------------------
  wiringPiSetup();
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH); 
  delay(500);
  // --------------------------------

  // --------------------------------
  // Camera Section 
  // (It has infinity loop)
  // --------------------------------
  RaspiCamera camera(320,240,0,0,0);
  while(cvWaitKey(1) != 27){
    camera.img = raspiCamCvQueryFrame(camera.cap);
    cvShowImage("RasPi Camera", camera.img);
  }
  // --------------------------------
  
  return 0 ;
}
