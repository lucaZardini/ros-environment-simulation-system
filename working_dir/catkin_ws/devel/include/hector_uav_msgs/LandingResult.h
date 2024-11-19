// Generated by gencpp from file hector_uav_msgs/LandingResult.msg
// DO NOT EDIT!


#ifndef HECTOR_UAV_MSGS_MESSAGE_LANDINGRESULT_H
#define HECTOR_UAV_MSGS_MESSAGE_LANDINGRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hector_uav_msgs
{
template <class ContainerAllocator>
struct LandingResult_
{
  typedef LandingResult_<ContainerAllocator> Type;

  LandingResult_()
    {
    }
  LandingResult_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::hector_uav_msgs::LandingResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_uav_msgs::LandingResult_<ContainerAllocator> const> ConstPtr;

}; // struct LandingResult_

typedef ::hector_uav_msgs::LandingResult_<std::allocator<void> > LandingResult;

typedef boost::shared_ptr< ::hector_uav_msgs::LandingResult > LandingResultPtr;
typedef boost::shared_ptr< ::hector_uav_msgs::LandingResult const> LandingResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_uav_msgs::LandingResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace hector_uav_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_uav_msgs::LandingResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_uav_msgs::LandingResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_uav_msgs::LandingResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::hector_uav_msgs::LandingResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_uav_msgs/LandingResult";
  }

  static const char* value(const ::hector_uav_msgs::LandingResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
;
  }

  static const char* value(const ::hector_uav_msgs::LandingResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LandingResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_uav_msgs::LandingResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::hector_uav_msgs::LandingResult_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_UAV_MSGS_MESSAGE_LANDINGRESULT_H
