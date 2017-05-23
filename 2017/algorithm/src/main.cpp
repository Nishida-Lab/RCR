#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/clamp.hpp>
#include <boost/geometry/util/math.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <algorithm/version.hpp>


template <typename T>
T angle(const boost::numeric::ublas::vector<T>& v,
        const boost::numeric::ublas::vector<T>& u)
{
  namespace ublas = boost::numeric::ublas;

  const T length {ublas::norm_2(v) * ublas::norm_2(u)};

  if (boost::geometry::math::equals(length, static_cast<T>(0.0)))
  {
    return boost::math::constants::half_pi<T>();
  }

  return std::acos(
           boost::algorithm::clamp(
             ublas::inner_prod(v, u) / length, static_cast<T>(-1.0), static_cast<T>(+1.0)
           )
         );
}


template <typename T>
T radian_to_degree(const T& radian)
{
  return radian * static_cast<T>(180.0) / boost::math::constants::pi<T>();
}


namespace robocar {


template <typename T>
class direction
  : public boost::numeric::ublas::vector<T>
{
  static constexpr std::size_t dimension_ {2};

  std::vector<boost::numeric::ublas::vector<T>> v_;

  std::fstream sensor_device_;

public:
  direction(const std::string& sensor_device)
    : boost::numeric::ublas::vector<T> {dimension_},
      v_ {8, boost::numeric::ublas::vector<T> {dimension_}},
      sensor_device_ {sensor_device}
  {
    (*this) <<= 0.0, 1.0;

    v_[3] <<=  1.0, -1.0;  v_[2] <<=  0.0, -1.0;  v_[1] <<= -1.0, -1.0;
    v_[4] <<=  1.0,  0.0;      /* robocar */      v_[0] <<= -1.0,  0.0;
    v_[5] <<=  1.0,  1.0;  v_[6] <<=  0.0,  1.0;  v_[7] <<= -1.0,  1.0;

    for (auto&& v : v_)
    {
      v = normalized(v);
      std::cout << "[debug] v_[" << std::noshowpos << &v - &v_.front() << "] "
                << std::fixed << std::setprecision(3) << std::showpos << v << std::endl;
    }

    if (!sensor_device_.is_open())
    {
      std::cerr << "[error] failed to open device \"" << sensor_device << "\"\n";
      std::exit(EXIT_FAILURE);
    }
  }

  void read_line()
  {
    static std::string buffer {};
    std::getline(sensor_device_, buffer, '\n');

    std::cout << "[debug] funtion read_line(): " << buffer << std::endl;
  }

protected:
  auto normalized(const boost::numeric::ublas::vector<T>& v)
    -> boost::numeric::ublas::vector<T>
  {
    return v / boost::numeric::ublas::norm_2(v);
  }
};


} // namespace robocar


int main(int argc, char** argv)
{
  std::cout << "[debug] boost version: " << boost_version << "\n\n";

  const std::vector<std::vector<std::string>> world_map {
    {{"F0"}, {"F1"}, {"F2"}, {"F3"}, {"F4"}, {"F5"}},
    {{"E0"}, {"E1"}, {"E2"}, {"E3"}, {"E4"}, {"E5"}},
    {{"D0"}, {"D1"}, {"D2"}, {"D3"}, {"D4"}, {"D5"}},
    {{"C0"}, {"C1"}, {"C2"}, {"C3"}, {"C4"}, {"C5"}},
    {{"B0"}, {"B1"}, {"B2"}, {"B3"}, {"B4"}, {"B5"}},
    {{"A0"}, {"A1"}, {"A2"}, {"A3"}, {"A4"}, {"A5"}}
  };

  robocar::direction<double> direction {"/dev/stdin"};

  direction.read_line();

  return 0;
}

