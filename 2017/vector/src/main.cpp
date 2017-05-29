#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include <vector/vector.hpp>


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
  robocar::direction<double> direction {"/dev/stdin", "/dev/stdout"};

  robocar::vector<double> v1 {0.707, 0.707};

  return 0;
}

