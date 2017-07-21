#ifndef INCLUDED_ROBOCAR_UTILITY_PAIRED_POINTS_HPP_
#define INCLUDED_ROBOCAR_UTILITY_PAIRED_POINTS_HPP_


#include <utility>


namespace robocar { namespace utility { namespace renamed_pair {


template <typename T, typename U = T>
class point
  : std::pair<T,U>
{
public:
  T& x;
  U& y;

  template <typename... Ts>
  explicit point(Ts&&... args)
    : std::pair<T,T> {std::forward<Ts>(args)...},
      x {(*this).first},
      y {(*this).second}
  {}
};


}}} // namespace robocar::utility::renamed_pair


#endif

