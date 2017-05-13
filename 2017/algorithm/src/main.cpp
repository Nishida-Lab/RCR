#include <iostream>
#include <string>
#include <vector>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <algorithm/version.hpp>


template <typename T>
auto normalize(const boost::numeric::ublas::vector<T>& v)
  -> boost::numeric::ublas::vector<T>
{
  return v / boost::numeric::ublas::norm_2(v);
}


int main(int argc, char** argv)
{
  const std::vector<std::vector<std::string>> world_map {
    {{"F0"}, {"F1"}, {"F2"}, {"F3"}, {"F4"}, {"F5"}},
    {{"E0"}, {"E1"}, {"E2"}, {"E3"}, {"E4"}, {"E5"}},
    {{"D0"}, {"D1"}, {"D2"}, {"D3"}, {"D4"}, {"D5"}},
    {{"C0"}, {"C1"}, {"C2"}, {"C3"}, {"C4"}, {"C5"}},
    {{"B0"}, {"B1"}, {"B2"}, {"B3"}, {"B4"}, {"B5"}},
    {{"A0"}, {"A1"}, {"A2"}, {"A3"}, {"A4"}, {"A5"}}
  };

  boost::numeric::ublas::vector<double> uv {2};

  uv[0] = 1.0;
  uv[1] = 0.0;

  std::cout << "[debug] ublas vector: " << uv << std::endl;

  return 0;
}

