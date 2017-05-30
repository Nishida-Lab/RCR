#ifndef INCLUDED_ROBOCAR_2017_CAMERA_COLOR_RANGE_HPP_
#define INCLUDED_ROBOCAR_2017_CAMERA_COLOR_RANGE_HPP_


#include <utility>


namespace robocar {


template <typename T>
class color_range
  : public std::pair<T,T>
{
public:
  template <typename... Ts>
  color_range(Ts&&... args)
    : std::pair<T,T> {std::forward<Ts>(args)...}
  {}

  T min() const noexcept
  {
    return (*this).first;
  }

  T max() const noexcept
  {
    return (*this).second;
  }
};


} // namespace robocar


#endif
