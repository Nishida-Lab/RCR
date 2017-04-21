
#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include <exception>
#include "main.hpp"
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;
/*
void GetMaskHSV(IplImage* src, IplImage* mask,int erosions, int dilations)
{
	int x = 0, y = 0;
	uchar H, S, V;
	uchar minH, minS, minV, maxH, maxS, maxV;

	CvPixelPosition8u pos_src, pos_dst;
	uchar* p_src;
	uchar* p_dst;
	IplImage* tmp;


	tmp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);

	//HSVに変換
	cvCvtColor(src, tmp, CV_RGB2HSV);

	CV_INIT_PIXEL_POS(pos_src, (unsigned char*) tmp->imageData,
					   tmp->widthStep,cvGetSize(tmp), x, y, tmp->origin);

	CV_INIT_PIXEL_POS(pos_dst, (unsigned char*) mask->imageData,
					   mask->widthStep, cvGetSize(mask), x, y, mask->origin);

	minH = 100;	maxH = 115;
	minS = 80;	maxS = 255;
	minV = 120;	maxV = 255;
	for(y = 0; y < tmp->height; y++) {
		for(x = 0; x < tmp->width; x++) {
			p_src = CV_MOVE_TO(pos_src, x, y, 3);
			p_dst = CV_MOVE_TO(pos_dst, x, y, 3);

			H = p_src[0];	//0から180
			S = p_src[1];
			V = p_src[2];

			if( minH <= H && H <= maxH &&
				minS <= S && S <= maxS &&
				minV <= V && V <= maxV
			) {
				p_dst[0] = 255;
				p_dst[1] = 255;
				p_dst[2] = 255;
			} else {
				p_dst[0] = 0;
				p_dst[1] = 0;
				p_dst[2] = 0;
			}
		}
	}

	if(erosions > 0)  cvErode(mask, mask, 0, erosions);
	if(dilations > 0) cvDilate(mask, mask, 0, dilations);

	cvReleaseImage(&tmp);
}
*/

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
    cv::Mat cameraMat = cv::cvarrToMat(camera.img);
    Mat hsv_img;
    Mat result;
    Mat hsv_skin_img;
    Mat smooth_img;


    //namedWindow("b_img",CV_WINDOW_AUTOSIZE);

    int height = cameraMat.rows;
    int width = cameraMat.cols;

    Mat b_img(cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
    Mat r_img(cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));


    //cv::medianBlur(cameraMat,smooth_img,7); //平滑化
    cv::cvtColor(cameraMat,hsv_img,CV_BGR2HSV);	//HSVに変換
    //  cv::cvtColor(b_img,blue_img,CV_BGR2HSV);
    cv::imshow("hsv",hsv_img);

    // barabara  
    vector<cv::Mat> planes;
    cv::split(hsv_img, planes);
    Mat h_img = planes[0];
    Mat s_img = planes[1];
    Mat v_img = planes[2];
    Mat h_img_c;
    int sum_bx = 0 ,sum_by = 0;
    int sum_rx = 0 ,sum_ry = 0;
    int countx = 0 ,county = 0;
    

    for(int y = 0 ; y < height; y++){
      for(int x = 0 ; x < width; x++){
	//abstraction blue in b_img
	int b = y*b_img.step + x*b_img.channels();
	if(95 <=h_img.at<uchar>(y,x) && h_img.at<uchar>(y,x) <= 135 &&
	   s_img.at<uchar>(y,x)>=50 && v_img.at<uchar>(y,x) >=50  ) {
	  b_img.data[b] = 255;
	  sum_bx += x;
	  //sum_by += y;
	  countx += 1;
	}
	//abstraction red in r_img
	if(165 <=h_img.at<uchar>(y,x) && h_img.at<uchar>(y,x) <= 180 &&
	   s_img.at<uchar>(y,x)>=50 && v_img.at<uchar>(y,x) >=50  ) {
	  r_img.data[b+2]=255;
	  sum_rx += x;
	  //sum_ry += y;
	  county += 1;
	}
      }
    }
    if(countx == 0 || county == 0)
      cout << " "<<endl;
    //keisan
    else if(sum_bx/countx > sum_rx/county)
      cout << "     "<<"右"<<endl;
    else
      cout <<"左"<<endl;
    
    //noise disposal
    //  Mat e4 = (Mat_<uchar>(3,3)<<0,1,0,1,1,1,0,1,0);
    //  morphologyEx(h_img,h_img_c,MORPH_CLOSE,e4,Point(-1,-1),5);

    

    // imshow("h_img", h_img);
    //   imshow("s_img", s_img);
    //imshow("v_img", v_img);
    //imshow("h_img.c",h_img_c);
    imshow("b_img",b_img);
    imshow("r_img",r_img);


    cv::imshow("RasPi Camera.Mat",cameraMat);
    //  cvShowImage("RasPi Camera", camera.img);
    
  }
  // --------------------------------
  
  return 0 ;
}
