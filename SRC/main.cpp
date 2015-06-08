#include <iostream>
#include <unistd.h>
#include "main.hpp"

using namespace std;

int main()
{
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
  motor.Drive(1.33, -0.410);
  sleep(5);
  motor.Drive(0,0);
  sleep(1);
  motor.Drive(-2.22, 0.410);
  sleep(5);

  return 0 ;
}
