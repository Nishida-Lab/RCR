#ifndef INCLUDED_ROBOCAR_UTILITY_RENAMED_PAIR_HPP_
#define INCLUDED_ROBOCAR_UTILITY_RENAMED_PAIR_HPP_


#include <utility>


#undef  ROBOCAR_RENAMED_PAIR_IMPLIMENT
#define ROBOCAR_RENAMED_PAIR_IMPLIMENT(NAME, FIRST, SECOND) \
template <typename T, typename U = T>                       \
class NAME                                                  \
  : std::pair<T,U>                                          \
{                                                           \
public:                                                     \
  T& FIRST;                                                 \
  U& SECOND;                                                \
                                                            \
  template <typename... Ts>                                 \
  explicit constexpr NAME(Ts&&... args)                     \
    : std::pair<T,U> {std::forward<Ts>(args)...},           \
      FIRST  {(*this).first},                               \
      SECOND {(*this).second}                               \
  {}                                                        \
};                                                          \


namespace robocar { namespace utility { namespace renamed_pair {


ROBOCAR_RENAMED_PAIR_IMPLIMENT(point, x, y)
ROBOCAR_RENAMED_PAIR_IMPLIMENT(color_range, min, max)
ROBOCAR_RENAMED_PAIR_IMPLIMENT(area, width, height)


}}} // namespace robocar::utility::renamed_pair


#endif

