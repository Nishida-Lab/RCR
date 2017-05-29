#ifndef INCLUDED_ROBOCAR_VECTOR_VECTOR_HPP_
#define INCLUDED_ROBOCAR_VECTOR_VECTOR_HPP_


#include <utility>

#include <boost/algorithm/clamp.hpp>
#include <boost/geometry/util/math.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>


namespace robocar {


template <typename T>
class vector
  : public boost::numeric::ublas::vector<T>
{
public:
  static constexpr std::size_t extents {2};

  explicit vector(T&& x = static_cast<T>(0), T&& y = static_cast<T>(0))
    : boost::numeric::ublas::vector<T> {extents}
  {
    *this <<= x, y;
  }

  explicit vector(const std::pair<T,T>& pair)
    : boost::numeric::ublas::vector<T> {extents}
  {
    *this <<= pair.first, pair.second;
  }
};


} // robocar


#endif
