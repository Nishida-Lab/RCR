#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <camera/version.hpp>
#include <camera/color_range.hpp>


#define CONSOLE_DEBUG
#undef  CONSOLE_DEBUG


namespace robocar {


class camera
  : public raspicam::RaspiCam_Cv
{
public:
  using image_type = cv::Mat;

private:
  image_type image_buffer_;

  const color_range<std::uint16_t> h_ { 30, 330};
  const color_range<std::uint16_t> s_ { 30, 100};
  const color_range<std::uint16_t> v_ { 50, 100};

public:
  camera(std::size_t width = 2592, std::size_t height = 1944)
    : raspicam::RaspiCam_Cv {},
      image_buffer_ {}
  {
    set(CV_CAP_PROP_FRAME_WIDTH,  width);
    set(CV_CAP_PROP_FRAME_HEIGHT, height);

#ifndef CONSOLE_DEBUG
    if (!open())
    {
      std::cerr << "[error] failed to open camera module\n";
      std::exit(EXIT_FAILURE);
    }

    std::cout << "[debug] connected to camera module: " << getId() << std::endl;
#endif
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
    cv::imwrite("hoge_rgb.jpg", image_buffer_);
    cv::imwrite("hoge_bin.jpg", hoge(image_buffer_));
  }

private:
  image_type hoge(const image_type& rgb)
  {
    image_type binary {cv::Mat::zeros(rgb.size(), CV_8UC1)};
    image_proc(rgb, binary);

    return binary;
  }

  void image_proc(const image_type& rgb, image_type& binary)
  {
    image_type blur {}, hsv {};

    cv::GaussianBlur(rgb, blur, cv::Size(5, 5), 4.0, 4.0);
    cv::cvtColor(blur, hsv, CV_BGR2HSV);

    // mask(hsv, binary);
    binary = red_mask(hsv);

    cv::dilate(binary, binary, cv::Mat {}, cv::Point(-1, -1), 2);
    cv::erode( binary, binary, cv::Mat {}, cv::Point(-1, -1), 4);
    cv::dilate(binary, binary, cv::Mat {}, cv::Point(-1, -1), 1);
  }

  cv::Mat1b red_mask(const cv::Mat3b& hsv)
  {
    static cv::Mat1b mask1 {}, mask2 {};

    cv::inRange(hsv, cv::Scalar(  0,  70,  50), cv::Scalar(  0, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(150,  70,  50), cv::Scalar(180, 255, 255), mask2);

    return cv::Mat1b {mask1 | mask2};
  }

  void mask(const image_type& hsv, image_type& binary)
  {
    for (int row {0}; row < hsv.rows; ++row)
    {
      for (int col {0}; col < hsv.cols; ++col)
      {
        std::size_t a {hsv.step * row + col * 3};

        if ((hsv.data[a] <= h_.min() || hsv.data[a] >= h_.max()) && (hsv.data[a+1] >= s_.min()) && (hsv.data[a+2] >= v_.min()))
        {
          binary.at<unsigned char>(row, col) = 255;
        }

        else
        {
          binary.at<unsigned char>(row, col) = 0;
        }
      }
    }
  }
};


} // namespace robocar


int main(int argc, char** argv)
{
  robocar::camera camera {1280, 960};

  camera.read();
  camera.write("hoge.jpg");

  return 0;
}

