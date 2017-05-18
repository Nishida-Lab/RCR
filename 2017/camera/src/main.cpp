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
  using image_type = cv::Mat;

private:
  image_type image_buffer_;

public:
  camera(std::size_t width = 2592, std::size_t height = 1944)
    : raspicam::RaspiCam_Cv {},
      image_buffer_ {}
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

  void read()
  {
    grab();
    retrieve(image_buffer_);
  }

  void write(const std::string& s)
  {
    cv::imwrite(s, image_buffer_);
  }
};



} // namespace robocar


int main(int argc, char** argv)
{
  robocar::camera camera {2592, 1944};

  camera.read();
  camera.write("hoge.jpg");

  return 0;
}

