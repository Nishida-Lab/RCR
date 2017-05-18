#include <cstdlib>
#include <iostream>
#include <string>

#include <raspicam/raspicam_cv.h>

#include <camera/version.hpp>


namespace robocar {


class camera
  : public raspicam::RaspiCam_Cv
{
public:
  camera(std::size_t width = 2592, std::size_t height = 1944)
    : raspicam::RaspiCam_Cv {}
  {
    set(CV_CAP_PROP_FRAME_WIDTH,  width);
    set(CV_CAP_PROP_FRAME_HEIGHT, height);

    if (!open())
    {
      std::cerr << "[error] failed to open camera module\n";
      std::exit(EXIT_FAILURE);
    }

    std::cout << "[debug] connected to camera module: " << getId() << std::endl;
  }

  ~camera()
  {
    release();
  }
};



} // namespace robocar


int main(int argc, char** argv)
{
  raspicam::RaspiCam_Cv camera {};
  camera.set(CV_CAP_PROP_FRAME_WIDTH,  2592);
  camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1944);

  std::string file_name {"test_image.jpg"};

  if (!camera.open())
  {
    std::cerr << "[debug] failed to open camera\n";
    std::exit(EXIT_FAILURE);
  }

  std::cout << "[debug] connected to camera: " << camera.getId() << std::endl;

  cv::Mat image {};

  camera.grab();
  camera.retrieve(image);

  cv::imwrite("test_image.jpg", image);
  std::cout << "[debug] image saved: " << file_name << std::endl;

  camera.release();

  return 0;
}

