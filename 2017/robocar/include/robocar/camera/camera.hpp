#ifndef INCLUDED_ROBOCAR_2017_CAMERA_CAMERA_HPP_
#define INCLUDED_ROBOCAR_2017_CAMERA_CAMERA_HPP_


#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

// #include <robocar/camera/color_range.hpp>
#include <robocar/vector/vector.hpp>
#include <robocar/utility/renamed_pair.hpp>


namespace robocar {


class camera
  : public raspicam::RaspiCam_Cv
{
  cv::Mat3b image_;

  const std::size_t width_, height_;

public:
  camera(std::size_t width = 2592, std::size_t height = 1944)
    : raspicam::RaspiCam_Cv {},
      width_  {width},
      height_ {height}
  {
    set(CV_CAP_PROP_FRAME_WIDTH, width_);
    set(CV_CAP_PROP_FRAME_HEIGHT, height_);

    if (!open())
    {
      std::cerr << "[error] failed to open camera module\n";
      std::exit(EXIT_FAILURE);
    }

    std::cout << "[debug] connected to camera module: " << getId() << std::endl;
  }

  ~camera() noexcept
  {
    release();
  }

  void read()
  {
    grab();
    retrieve(image_);
    // image_ = cv::Mat3b {image_, cv::Rect {0, static_cast<int>(height_ * 0.5), static_cast<int>(width_), static_cast<int>(height_ * 0.5)}};
  }

  // void debug(const std::string& prefix = "debug_")
  // {
  //   read();
  //
  //   std::ofstream ofs {prefix + "props.txt"/*, std::ios::out*/};
  //
  //   if (!ofs)
  //   {
  //     std::cout << "[error] failed to open file: " << prefix + "props.txt\n";
  //     std::exit(EXIT_FAILURE);
  //   }
  //
  //   ofs << "brightness:         " << get(CV_CAP_PROP_BRIGHTNESS)
  //       << "contrast:           " << get(CV_CAP_PROP_CONTRAST)
  //       << "saturation:         " << get(CV_CAP_PROP_SATURATION)
  //       << "gain:               " << get(CV_CAP_PROP_GAIN)
  //       << "exposure:           " << get(CV_CAP_PROP_EXPOSURE)
  //       << "white balance red:  " << get(CV_CAP_PROP_WHITE_BALANCE_RED_V)
  //       << "white balance blue: " << get(CV_CAP_PROP_WHITE_BALANCE_BLUE_U)
  //       << std::endl;
  //
  //   cv::imwrite(prefix + "1_raw.jpg", image_);
  //
  //   cv::Mat3b hsv {convert(image_)};
  //   cv::imwrite(prefix + "2_hsv.jpg", hsv);
  //
  //   cv::Mat1b red_masked {red_mask(hsv)};
  //   cv::imwrite(prefix + "3_red_masked.jpg", red_masked);
  //
  //   cv::Mat1b red_opened {opening(red_masked)};
  //   cv::imwrite(prefix + "4_red_opened.jpg", red_opened);
  //
  //   cv::Mat1b contour {find_contours_debug(red_opened)};
  //   cv::imwrite(prefix + "5_contour.jpg", contour);
  // }

  auto find()
    -> std::vector<std::pair<std::size_t,std::size_t>>
  {
    read();
    return find_contours(opening(red_mask(convert(image_))));
  }

  template <typename T>
  auto search()
    -> std::vector<robocar::vector<T>>
  {
    std::vector<robocar::vector<T>> poles {};

    for (const auto& p : find())
    {
      int x_pixel {static_cast<int>(p.first) - static_cast<int>(width_ / 2)};
      T x_ratio {static_cast<double>(x_pixel) / static_cast<double>(width_ / 2)};

      poles.emplace_back(x_ratio, std::pow(static_cast<double>(1.0) - std::pow(x_ratio, 2.0), 0.5));
    }

    std::sort(poles.begin(), poles.end(), [&](auto a, auto b) {
      return std::abs(a[0]) < std::abs(b[0]);
    });

    return poles;
  }

private:
  cv::Mat3b convert(const cv::Mat3b& rgb) const
  {
    cv::Mat3b hsv {};
    cv::cvtColor(rgb, hsv, CV_BGR2HSV);

    return hsv;
  }

  cv::Mat1b opening(const cv::Mat1b& bin)
  {
    cv::Mat1b result {};
    cv::morphologyEx(bin, result, CV_MOP_CLOSE, cv::Mat1b {}, cv::Point {-1, -1}, 2);

    return result;
  }

  cv::Mat1b red_mask(const cv::Mat3b& hsv) // TODO move to ctor
  {
    static cv::Mat1b mask1 {}, mask2 {};

    cv::inRange(hsv, cv::Scalar {  0, 170, 70}, cv::Scalar { 20, 255, 255}, mask1);
    cv::inRange(hsv, cv::Scalar {160, 170, 70}, cv::Scalar {179, 255, 255}, mask2);

    return cv::Mat1b {mask1 | mask2};
  }

  // auto find_contours_debug(const cv::Mat& bin) const
  //   -> cv::Mat
  // {
  //   std::vector<std::vector<cv::Point>> contours {};
  //               std::vector<cv::Point>  pole_moments {};
  //
  //   cv::Mat result {bin};
  //
  //   cv::findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  //
  //   static constexpr double pole_ratio {static_cast<double>(2) / static_cast<double>(3)}; // XXX magic number
  //   static constexpr double tolerance {0.20}; // percent
  //
  //   std::cout << "[debug] pole ratio: " << pole_ratio << std::endl;
  //   std::cout << "        " << pole_ratio * (1 - tolerance) << " < range < " << pole_ratio * (1 + tolerance) << std::endl;
  //
  //   for (auto iter = contours.begin(); iter != contours.end(); ++iter)
  //   {
  //     auto rect = cv::boundingRect(*iter); // bounding box
  //     double rect_ratio {static_cast<double>(rect.width) / static_cast<double>(rect.height)};
  //
  //     std::cout << "[debug] rect ratio: " << rect_ratio << std::endl;
  //
  //     if (pole_ratio * (1 - tolerance) < rect_ratio && rect_ratio < pole_ratio * (1 + tolerance))
  //     {
  //       cv::rectangle(result, cv::Point {rect.x, rect.y}, cv::Point {rect.x + rect.width, rect.y + rect.height},
  //                     cv::Scalar {255, 0, 0}, 3, CV_AA);
  //
  //       cv::Moments moment {cv::moments(*iter)};
  //       pole_moments.emplace_back(moment.m10 / moment.m00, moment.m01 / moment.m00);
  //     }
  //
  //     else
  //     {
  //       cv::rectangle(result, cv::Point {rect.x, rect.y}, cv::Point {rect.x + rect.width, rect.y + rect.height},
  //                     cv::Scalar {255, 0, 0}, 1, CV_AA);
  //     }
  //   }
  //
  //   for (const auto& pm : pole_moments)
  //   {
  //     std::cout << "[debug] maybe point of pole moment: " << pm << std::endl;
  //
  //     static constexpr int radius {4};
  //     static constexpr int thickness {-1};
  //     cv::circle(result, pm, radius, cv::Scalar {255, 0, 0}, thickness);
  //   }
  //
  //   return result;
  // }

  auto find_contours(const cv::Mat1b& bin) const
    -> std::vector<std::pair<std::size_t,std::size_t>>
  {
    std::vector<std::vector<cv::Point>> contours {};
    std::vector<std::pair<std::size_t,std::size_t>> pole_moments {};

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
