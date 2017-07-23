#ifndef INCLUDED_ROBOCAR_SENSOR_SENSOR_NODE_HPP_
#define INCLUDED_ROBOCAR_SENSOR_SENSOR_NODE_HPP_


#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <robocar/sensor/wiring_serial.hpp>


namespace robocar {


template <typename C> class sensor_node;
template <typename C> using sensor_edge = std::unique_ptr<robocar::sensor_node<C>>;


template <typename C>
class [[deprecated]] sensor_node
  : public robocar::wiring_serial<C>,
    public std::unordered_map<std::basic_string<C>, robocar::sensor_edge<C>>
{
public:
  template <typename... Ts>
  explicit sensor_node(Ts&&... args)
    : robocar::wiring_serial<C> {std::forward<Ts>(args)...}
  {}

  auto& operator[](const std::basic_string<C>& node_name)
  {
    if ((*this).find(node_name) == (*this).end())
    {
      robocar::sensor_edge<C> edge {new robocar::sensor_node<C> {*this}};
      (*this).emplace(node_name, std::move(edge));
    }

    return *(*this).at(node_name);
  }
};


} // namespace robocar


#endif

