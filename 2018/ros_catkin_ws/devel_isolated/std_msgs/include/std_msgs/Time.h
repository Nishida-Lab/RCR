// Generated by gencpp from file std_msgs/Time.msg
// DO NOT EDIT!


#ifndef STD_MSGS_MESSAGE_TIME_H
#define STD_MSGS_MESSAGE_TIME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace std_msgs
{
template <class ContainerAllocator>
struct Time_
{
  typedef Time_<ContainerAllocator> Type;

  Time_()
    : data()  {
    }
  Time_(const ContainerAllocator& _alloc)
    : data()  {
  (void)_alloc;
    }



   typedef ros::Time _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::std_msgs::Time_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::std_msgs::Time_<ContainerAllocator> const> ConstPtr;

}; // struct Time_

typedef ::std_msgs::Time_<std::allocator<void> > Time;

typedef boost::shared_ptr< ::std_msgs::Time > TimePtr;
typedef boost::shared_ptr< ::std_msgs::Time const> TimeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::std_msgs::Time_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::std_msgs::Time_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace std_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/home/pi/RCR/2018/ros_catkin_ws/src/std_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Time_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::std_msgs::Time_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Time_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::std_msgs::Time_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Time_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::std_msgs::Time_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::std_msgs::Time_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cd7166c74c552c311fbcc2fe5a7bc289";
  }

  static const char* value(const ::std_msgs::Time_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcd7166c74c552c31ULL;
  static const uint64_t static_value2 = 0x1fbcc2fe5a7bc289ULL;
};

template<class ContainerAllocator>
struct DataType< ::std_msgs::Time_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Time";
  }

  static const char* value(const ::std_msgs::Time_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::std_msgs::Time_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time data\n\
";
  }

  static const char* value(const ::std_msgs::Time_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::std_msgs::Time_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Time_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::std_msgs::Time_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::std_msgs::Time_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STD_MSGS_MESSAGE_TIME_H
