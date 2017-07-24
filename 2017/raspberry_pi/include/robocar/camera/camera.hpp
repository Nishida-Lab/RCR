#ifndef INCLUDED_ROBOCAR_CAMERA_CAMERA_HPP_
#define INCLUDED_ROBOCAR_CAMERA_CAMERA_HPP_


#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <robocar/vector/vector.hpp>
#include <robocar/utility/renamed_pair.hpp>


namespace robocar {


class camera
  : public raspicam::RaspiCam_Cv
{
  cv::Mat3b buffer_;

public:
  static constexpr robocar::utility::renamed_pair::area<std::size_t> size_max {2592, 1944};
         const     robocar::utility::renamed_pair::area<std::size_t> size;

public:
  camera(std::size_t width = size_max.width, std::size_t height = size_max.height)
    : raspicam::RaspiCam_Cv {},
      size {width, height}
  {
    set(CV_CAP_PROP_FRAME_WIDTH,  size.width);
    set(CV_CAP_PROP_FRAME_HEIGHT, size.height);
    set(CV_CAP_PROP_GAIN,                 50); // values range from 0 to 100
    set(CV_CAP_PROP_EXPOSURE,             50); // -1 is auto, values range from 0 to 100
    set(CV_CAP_PROP_WHITE_BALANCE_RED_V,  50); // values range from 0 to 100, -1 auto whitebalance
    set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 50); // values range from 0 to 100, -1 auto whitebalance

    if (!raspicam::RaspiCam_Cv::open())
    {
      std::cerr << "[error] failed to open camera module\n";
      std::exit(EXIT_FAILURE);
    }

    std::cout << "[debug] connected to camera module: " << getId() << std::endl;
  }

  ~camera() noexcept
  {
    raspicam::RaspiCam_Cv::release();
  }

  const auto& read()
  {
    raspicam::RaspiCam_Cv::grab();
    raspicam::RaspiCam_Cv::retrieve(buffer_);
    return buffer_;
  }

  template <typename FilterFunction>
  decltype(auto) capture(FilterFunction&& func)
  {
    return func(std::forward<decltype(read())>(read()));
  }

  [[deprecated]] auto find()
  {
    read();
    return find_contours(morphology(red_mask(convert(buffer_))));
  }

  template <typename T>
  [[deprecated]] auto search()
    -> std::vector<robocar::vector<T>>
  {
    std::vector<robocar::vector<T>> poles {};

    for (const auto& p : find())
    {
      int x_pixel {static_cast<int>(p.x) - static_cast<int>(size.width / 2)};
      T x_ratio {static_cast<double>(x_pixel) / static_cast<double>(size.width / 2)};

      poles.emplace_back(x_ratio, std::pow(static_cast<double>(1.0) - std::pow(x_ratio, 2.0), 0.5));
    }

    std::sort(poles.begin(), poles.end(), [&](auto a, auto b) {
      return std::abs(a[0]) < std::abs(b[0]);
    });

    return poles;
  }

private:
  [[deprecated]] cv::Mat3b& convert(const cv::Mat3b& rgb) const
  {
    static cv::Mat3b result {};
    cv::cvtColor(rgb, result, CV_BGR2HSV);

    return result;
  }

  [[deprecated]] cv::Mat1b& morphology(const cv::Mat1b& bin)
  {
    static cv::Mat1b result {};
    cv::morphologyEx(bin, result, CV_MOP_CLOSE, cv::Mat1b {}, cv::Point {-1, -1}, 2);

    return result;
  }

  [[deprecated]] cv::Mat1b& red_mask(const cv::Mat3b& hsv) // TODO move to ctor
  {
    static cv::Mat1b mask1 {}, mask2 {}, result;

    cv::inRange(hsv, cv::Scalar {  0, 170, 70}, cv::Scalar { 20, 255, 255}, mask1);
    cv::inRange(hsv, cv::Scalar {160, 170, 70}, cv::Scalar {179, 255, 255}, mask2);

    return result = mask1 | mask2;
  }

  auto find_contours(const cv::Mat1b& bin) const
    -> std::vector<robocar::utility::renamed_pair::point<std::size_t>>&
  {
    static std::vector<std::vector<cv::Point>> contours {};

    static std::vector<
      robocar::utility::renamed_pair::point<std::size_t>
    > pole_moments {};

    cv::findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    static constexpr double pole_ratio {static_cast<double>(2) / static_cast<double>(3)}; // XXX magic number
    static constexpr double tolerance {0.20}; // percent

    for (auto iter = contours.begin(); iter != contours.end(); ++iter)
    {
      auto rect = cv::boundingRect(*iter); // bounding box
      double rect_ratio {static_cast<double>(rect.width) / static_cast<double>(rect.height)};

      if (pole_ratio * (1 - tolerance) < rect_ratio && rect_ratio < pole_ratio * (1 + tolerance))
      {
        cv::Moments moment {cv::moments(*iter)};
        pole_moments.emplace_back(moment.m10 / moment.m00, moment.m01 / moment.m00);
      }
    }

    return pole_moments;
  }
};


} // namespace robocar


#endif

