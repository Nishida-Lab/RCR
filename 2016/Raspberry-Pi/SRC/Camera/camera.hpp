#ifndef __CAMERA__
#define __CAMERA__

#include <cv.h>
#include <highgui.h>
#include <RaspiCamCV.h>

class RaspiCamera
{
public:
  RaspiCamera();					// Constractor (void)
  RaspiCamera(int w, int h, int b, int f, int m);	// Constractor with params
  ~RaspiCamera();					//Destractor

  RaspiCamCvCapture *cap;				// For Capture
  IplImage *img;					// For OpenCV Image
 
private:
  RASPIVID_CONFIG *config;				//For Settings of Raspberry Pi Camera
  int width;						// width of camera image
  int height;						// height of camera image
  int bitrate;						// bitrate of camera image
  int framerate;					// frame rate of camera image
  int monochrome;					// wether monochrome or not

};

#endif
