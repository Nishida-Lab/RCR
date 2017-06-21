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
  static constexpr std::size_t extents {2}; // TODO recursive extents setting

  vector(T x = static_cast<T>(0), T y = static_cast<T>(0))
    : boost::numeric::ublas::vector<T> {extents}
  {
    *this <<= x, y;
  }

  vector(const std::pair<T,T>& pair)
    : boost::numeric::ublas::vector<T> {extents}
  {
    *this <<= pair.first, pair.second;
  }

public:
  static T radian_to_degree(const T& radian)
  {
    return radian * static_cast<T>(180.0) / boost::math::constants::pi<T>();
  }

  static T degree_to_radian(const T& degree)
  {
    return degree * boost::math::constants::pi<T>() / static_cast<T>(180.0);
  }

  template <template <typename...> class V = boost::numeric::ublas::vector>
  static T angle(const V<T>& v, const V<T>& u)
  {
    namespace ublas = boost::numeric::ublas;
    const T length {ublas::norm_2(v) * ublas::norm_2(u)};

    if (boost::geometry::math::equals(length, static_cast<T>(0)))
    {
      return boost::math::constants::half_pi<T>();
    }

    return std::acos(boost::algorithm::clamp(ublas::inner_prod(v, u) / length, static_cast<T>(-1.0), static_cast<T>( 1.0)));
  }

  template <template <typename...> class V = boost::numeric::ublas::vector>
  static V<T> normalize(const V<T>& v)
  {
    return {v / boost::numeric::ublas::norm_2(v)};
  }
};


} // robocar


#endif
