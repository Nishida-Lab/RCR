#ifndef INCLUDED_ROBOCAR_CAMERA_CAMERA_HPP_
#define INCLUDED_ROBOCAR_CAMERA_CAMERA_HPP_


#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <raspicam/raspicam_cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    cv::imwrite("debug/origin_image.jpg", buffer_);
    return buffer_;
  }

  template <typename FilterFunction, typename... Ts>
  decltype(auto) capture(FilterFunction&& func, Ts&&... args)
  {
    return func(std::forward<decltype(read())>(read()), std::forward<Ts>(args)...);
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

  [[deprecated]] auto find_contours(const cv::Mat1b& bin) const
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

public:
  static auto untested_filter(const cv::Mat3b& origin_image, std::size_t& width, std::uint8_t hue)
  {
    const cv::Mat3b cutted_image {
      origin_image,
      cv::Rect {
        0,
        static_cast<int>(origin_image.size().height * 0.5),
        origin_image.size().width,
        static_cast<int>(origin_image.size().height * 0.5)
      }
    };
#ifndef NDEBUG
    cv::imwrite("debug/cutted_image.jpg", cutted_image);
#endif

    static cv::Mat3b hsv_converted_image {};
    cv::cvtColor(cutted_image, hsv_converted_image, cv::COLOR_BGR2HSV);
#ifndef NDEBUG
    cv::imwrite("debug/hsv_converted_image.jpg", hsv_converted_image);
#endif

    static std::vector<cv::Mat1b> splited_images {};
    cv::split(hsv_converted_image, splited_images);

    cv::Mat1b& result_image {splited_images[0]};

    for (auto&& pixel : result_image)
    {
      pixel += (pixel < 90 ? 90 : -89);
    }
#ifndef NDEBUG
    cv::imwrite("debug/spinned_image.jpg", result_image);
#endif

    int average {
      std::accumulate(std::begin(result_image), std::end(result_image), 0)
        / (result_image.size().width * result_image.size().width)
    };

    for (std::size_t iter {0}; iter < 5; ++iter)
    {
      emphasize(result_image, average + hue, 0, 179);
    }

    for (auto&& pixel : result_image)
    {
      pixel = (pixel < 2 ? 179 : pixel);
    }
#ifndef NDEBUG
    cv::imwrite("debug/filtered_image.jpg", result_image);
#endif

    cv::Mat1b edge_image {};
    cv::morphologyEx(result_image, result_image, cv::MORPH_CLOSE, cv::Mat1b {}, cv::Point {-1, -1}, 3);
    cv::Canny(result_image, edge_image, 50, 200);

    static std::vector<std::vector<cv::Point>> contours {};
    cv::findContours(edge_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

#ifndef NDEBUG
    std::cout << "[debug] average: " << average << std::endl;
    std::cout << "[debug] contour: " << contours.size() << std::endl;
#endif

    if (contours.empty())
    {
      return robocar::vector<double> {0.0, 0.0};
    }

    std::pair<double, decltype(contours.front())> area_max {
      cv::contourArea(contours.front()), contours.front()
    };

    if (1 < contours.size())
    {
      for (auto iter = std::begin(contours) + 1; iter != std::end(contours); ++iter)
      {
        const double area {cv::contourArea(*iter)};
        if (area_max.first < area)
        {
          area_max.first = area;
          area_max.second = *iter;
        }
      }
    }

    auto moment = cv::moments(area_max.second);
    robocar::utility::renamed_pair::point<std::size_t> point {
      static_cast<int>(moment.m10 / moment.m00), static_cast<int>(moment.m01 / moment.m00)
    };

    const int pixel {static_cast<int>(point.x) - static_cast<int>(width / 2)};
    const double ratio {static_cast<double>(pixel) / static_cast<double>(width / 2)};

#ifndef NDEBUG
    cv::Mat3b contours_image {cutted_image};
    cv::drawContours(contours_image,
                     std::vector<std::vector<cv::Point>> {area_max.second},
                     -1, cv::Scalar {0, 255, 0}, CV_FILLED);
    cv::circle(contours_image,
               cv::Point {static_cast<int>(point.x), static_cast<int>(point.y)},
               10, cv::Scalar {0, 0, 255});
    cv::imwrite("debug/contours_image.jpg", contours_image);
#endif

    return robocar::vector<double> {
      ratio,
      std::pow(static_cast<double>(1.0) - std::pow(ratio, 2.0), 0.5)
    };
  }

  template <typename T, typename U>
  static void emphasize(T& image, U&& target_value,
                                  U&& min = std::numeric_limits<U>::min(),
                                  U&& max = std::numeric_limits<U>::max())
  {
    for (auto&& pixel : image)
    {
      if (pixel < target_value)
      {
        double distance {static_cast<double>(target_value - pixel - 1)};
        pixel = std::max(pixel * (1.0 - distance / target_value), static_cast<double>(min));
      }
      else
      {
        double distance {static_cast<double>(pixel - target_value)};
        pixel = std::min(pixel * (1.0 + distance / target_value), static_cast<double>(max));
      }
    }
  }
};


} // namespace robocar


#endif

