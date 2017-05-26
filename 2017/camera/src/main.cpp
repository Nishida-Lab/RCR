#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

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

  void debug(const std::string& prefix = "debug_")
  {
    read();

    cv::imwrite(prefix + "1_raw.jpg", image_buffer_);

    image_type hsv {convert(image_buffer_)};
    // cv::imwrite(prefix + "hsv.jpg", hsv);

    image_type red_masked {red_mask(hsv)};
    cv::imwrite(prefix + "2.1_red_masked.jpg", red_masked);

    image_type red_opened {opening(red_masked)};
    cv::imwrite(prefix + "2.2_red_opened.jpg", red_opened);

    image_type blue_masked {blue_mask(hsv)};
    cv::imwrite(prefix + "3.1_blue_masked.jpg", blue_masked);

    image_type blue_opened {opening(blue_masked)};
    cv::imwrite(prefix + "3.2_blue_opened.jpg", blue_opened);

    // image_type contour {find_contours(opened)};
    // cv::imwrite(prefix + "4_contour.jpg", contour);
  }

  // void write(const std::string& s)
  // {
  //   cv::imwrite("hoge_rgb.jpg", image_buffer_);
  //   cv::imwrite("hoge_bin.jpg", hoge(image_buffer_));
  // }

private:
  auto convert(const image_type& rgb)
    -> image_type
  {
    image_type hsv {};
    cv::cvtColor(rgb, hsv, CV_BGR2HSV);

    return hsv;
  }

  auto opening(const image_type& bin)
    -> image_type
  {
    image_type res {};

    cv::dilate(bin, res, cv::Mat {}, cv::Point(-1, -1), 2);
    cv::erode( res, res, cv::Mat {}, cv::Point(-1, -1), 4);
    cv::dilate(res, res, cv::Mat {}, cv::Point(-1, -1), 2);
    cv::erode( res, res, cv::Mat {}, cv::Point(-1, -1), 4);

    return res;
  }

  void image_proc(const image_type& rgb, image_type& binary)
  {
    image_type hsv {convert(rgb)};

    binary = red_mask(hsv);

    cv::dilate(binary, binary, cv::Mat {}, cv::Point(-1, -1), 2);
    cv::erode( binary, binary, cv::Mat {}, cv::Point(-1, -1), 4);
    cv::dilate(binary, binary, cv::Mat {}, cv::Point(-1, -1), 1);
  }

  cv::Mat1b red_mask(const cv::Mat3b& hsv) // TODO move to ctor
  {
    static cv::Mat1b mask1 {}, mask2 {};

    cv::inRange(hsv, cv::Scalar {  0, 100, 100}, cv::Scalar {  5, 255, 255}, mask1);
    cv::inRange(hsv, cv::Scalar {175, 100, 100}, cv::Scalar {179, 255, 255}, mask2);

    return cv::Mat1b {mask1 | mask2};
  }

  cv::Mat1b blue_mask(const cv::Mat3b& hsv)
  {
    static cv::Mat1b mask {};

    cv::inRange(hsv, cv::Scalar {100, 100, 100}, cv::Scalar {140, 255, 255}, mask);

    return cv::Mat1b {mask};
  }

  auto find_contours(const cv::Mat1b& bin) const
    -> cv::Mat1b
  {
    std::vector<std::vector<cv::Point>> contours {};
    decltype(bin) result {bin};

    cv::findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    // cv::findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (std::size_t i {0}; i < contours.size(); ++i)
    {
      cv::drawContours(result, contours, i, cv::Scalar(255, 0, 0), 3);
    }

    return result;
  }
};


} // namespace robocar


int main(int argc, char** argv)
{
  robocar::camera camera {1280, 960};

  // camera.read();
  // camera.write("hoge.jpg");

  camera.debug();

  return 0;
}

