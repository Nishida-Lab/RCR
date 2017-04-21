#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include <exception>
#include "main.hpp"

using namespace std;

int main()
{
  // ---------------------------------
  // Motor Swction (PWM Test Program)
  // ---------------------------------
  // try {
  //   Motor motor;  
    
  //   for(float i=1.0; i>=-1.0; i-=0.01){
  //     motor.Drive(1.0,i);
  //     usleep(100000);
  //   }

  // }catch(exception &e){
  //   cerr << e.what() << endl;
  //   cerr << "Stop the Program..." << endl;
  //   exit(1);
  // }
  // ---------------------------------

  
  // -----------------------------------------------
  // Camera Section (RaspiCam Capture Test Program)
  // -----------------------------------------------
  RaspiCamera camera(320,240,0,0,0);
  while(cvWaitKey(1) != 27){
    camera.img = raspiCamCvQueryFrame(camera.cap);
    cvShowImage("RasPi Camera", camera.img);
  }
  // --------------------------------
  
  return 0 ;
}
