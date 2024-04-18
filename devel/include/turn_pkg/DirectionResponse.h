// Generated by gencpp from file turn_pkg/DirectionResponse.msg
// DO NOT EDIT!


#ifndef TURN_PKG_MESSAGE_DIRECTIONRESPONSE_H
#define TURN_PKG_MESSAGE_DIRECTIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace turn_pkg
{
template <class ContainerAllocator>
struct DirectionResponse_
{
  typedef DirectionResponse_<ContainerAllocator> Type;

  DirectionResponse_()
    {
    }
  DirectionResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::turn_pkg::DirectionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turn_pkg::DirectionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DirectionResponse_

typedef ::turn_pkg::DirectionResponse_<std::allocator<void> > DirectionResponse;

typedef boost::shared_ptr< ::turn_pkg::DirectionResponse > DirectionResponsePtr;
typedef boost::shared_ptr< ::turn_pkg::DirectionResponse const> DirectionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::turn_pkg::DirectionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::turn_pkg::DirectionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace turn_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::turn_pkg::DirectionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::turn_pkg::DirectionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turn_pkg::DirectionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::turn_pkg::DirectionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "turn_pkg/DirectionResponse";
  }

  static const char* value(const ::turn_pkg::DirectionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
;
  }

  static const char* value(const ::turn_pkg::DirectionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DirectionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::turn_pkg::DirectionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::turn_pkg::DirectionResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // TURN_PKG_MESSAGE_DIRECTIONRESPONSE_H
