#ifndef INCLUDED_ROBOCAR_2017_CAMERA_CAMERA_HPP_
#define INCLUDED_ROBOCAR_2017_CAMERA_CAMERA_HPP_


#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <camera/version.hpp>
#include <camera/color_range.hpp>


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
    cv::imwrite(prefix + "2_red_masked.jpg", red_masked);

    image_type red_opened {opening(red_masked)};
    cv::imwrite(prefix + "3_red_opened.jpg", red_opened);

    image_type contour {find_contours_debug(red_opened)};
    cv::imwrite(prefix + "4_contour.jpg", contour);
  }

  auto find()
    -> std::vector<std::pair<std::size_t,std::size_t>>
  {
    read();

    return find_contours(opening(red_mask(convert(image_buffer_))));
  }

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

    return res;
  }

  cv::Mat1b red_mask(const cv::Mat3b& hsv) // TODO move to ctor
  {
    static cv::Mat1b mask1 {}, mask2 {};

    cv::inRange(hsv, cv::Scalar {  0, 100, 100}, cv::Scalar { 10, 255, 255}, mask1);
    cv::inRange(hsv, cv::Scalar {170, 100, 100}, cv::Scalar {179, 255, 255}, mask2);

    return cv::Mat1b {mask1 | mask2};
  }

  auto find_contours_debug(const cv::Mat& bin) const
    -> cv::Mat
  {
    std::vector<std::vector<cv::Point>> contours {};
                std::vector<cv::Point>  pole_moments {};

    cv::Mat result {bin};

    cv::findContours(bin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    static constexpr double pole_ratio {static_cast<double>(2) / static_cast<double>(3)}; // XXX magic number
    static constexpr double tolerance {0.20}; // percent

    std::cout << "[debug] pole ratio: " << pole_ratio << std::endl;
    std::cout << "        " << pole_ratio * (1 - tolerance) << " < range < " << pole_ratio * (1 + tolerance) << std::endl;

    for (auto iter = contours.begin(); iter != contours.end(); ++iter)
    {
      auto rect = cv::boundingRect(*iter); // bounding box
      double rect_ratio {static_cast<double>(rect.width) / static_cast<double>(rect.height)};

      std::cout << "[debug] rect ratio: " << rect_ratio << std::endl;

      if (pole_ratio * (1 - tolerance) < rect_ratio && rect_ratio < pole_ratio * (1 + tolerance))
      {
        cv::rectangle(result, cv::Point {rect.x, rect.y}, cv::Point {rect.x + rect.width, rect.y + rect.height},
                      cv::Scalar {255, 0, 0}, 1, CV_AA);

        cv::Moments moment {cv::moments(*iter)};
        pole_moments.emplace_back(moment.m10 / moment.m00, moment.m01 / moment.m00);
      }

      else
      {
        cv::rectangle(result, cv::Point {rect.x, rect.y}, cv::Point {rect.x + rect.width, rect.y + rect.height},
                      cv::Scalar {255, 0, 0}, 3, CV_AA);
      }
    }

    for (const auto& pm : pole_moments)
    {
      std::cout << "[debug] maybe point of pole moment: " << pm << std::endl;

      static constexpr int radius {4};
      static constexpr int thickness {-1};
      cv::circle(result, pm, radius, cv::Scalar {255, 0, 0}, thickness);
    }

    return result;
  }

  auto find_contours(const cv::Mat& bin) const
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
