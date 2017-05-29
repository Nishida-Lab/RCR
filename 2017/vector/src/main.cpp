#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include <boost/algorithm/clamp.hpp>
#include <boost/geometry/util/math.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>


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


template <typename T, std::size_t S = 2>
class direction
  : public boost::numeric::ublas::vector<T>
{
  static constexpr std::size_t dimension_ {S};

  std::vector<boost::numeric::ublas::vector<T>> v_;

  std::ifstream devin_;
  std::ofstream devout_;

public:
  direction(const std::string& devin_path, const std::string& devout_path)
    : boost::numeric::ublas::vector<T> {dimension_},
      v_ {8, boost::numeric::ublas::vector<T> {dimension_}},
      devin_ {devin_path},
      devout_ {devout_path}
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

    if (!devin_.is_open())
    {
      std::cerr << "[error] failed to open device \"" << devin_path << "\"\n";
      std::exit(EXIT_FAILURE);
    }

    if (!devout_.is_open())
    {
      std::cerr << "[error] failed to open device \"" << devout_path << "\"\n";
      std::exit(EXIT_FAILURE);
    }
  }

  std::string read()
  {
    static std::string buffer {};
    std::getline(devin_, buffer, '\n');

    return buffer;
  }

  std::string query(const std::string& prefix) // TODO timeout
  {
#ifdef DEBUG
    if (prefix[0] == 's')
    {
      return std::to_string(dummy_sensor_output(0, 20));
    }

    else if (prefix[0] == 'l')
    {
      return std::to_string(dummy_sensor_output(20, 180));
    }

    else if (prefix[0] == 'a')
    {
      return std::to_string(static_cast<int>(dummy_sensor_output(0, 1023))); // XXX
    }
#endif
    return std::to_string(dummy_sensor_output());
  }

protected:
  auto normalized(const boost::numeric::ublas::vector<T>& v)
    -> boost::numeric::ublas::vector<T>
  {
    return v / boost::numeric::ublas::norm_2(v);
  }

  T dummy_sensor_output(T&& min = static_cast<T>(0.0), T&& max = static_cast<T>(1.0))
  {
    static std::default_random_engine engine {std::random_device {}()};
    std::uniform_real_distribution<T> uniform {min, max};

    return uniform(engine);
  }
};


} // namespace robocar


int main(int argc, char** argv)
{
  std::cout << "[debug] boost version: " << boost_version << "\n\n";

  robocar::direction<double> direction {"/dev/stdin", "/dev/stdout"};
  std::putchar('\n');

  while (true)
  {
    std::cout << "[input] manual sensor data query: ";

    static std::string buffer {};
    std::cin >> buffer;

    std::cout << "[debug] query: " << buffer << ", result: " << direction.query(buffer) << std::endl;
  }

  return 0;
}

