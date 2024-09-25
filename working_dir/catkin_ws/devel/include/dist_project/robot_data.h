// Generated by gencpp from file dist_project/robot_data.msg
// DO NOT EDIT!


#ifndef DIST_PROJECT_MESSAGE_ROBOT_DATA_H
#define DIST_PROJECT_MESSAGE_ROBOT_DATA_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dist_project
{
template <class ContainerAllocator>
struct robot_data_
{
  typedef robot_data_<ContainerAllocator> Type;

  robot_data_()
    : robot_id()
    , target_height(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , sigma_x(0.0)
    , sigma_y(0.0)
    , sigma_theta(0.0)  {
    }
  robot_data_(const ContainerAllocator& _alloc)
    : robot_id(_alloc)
    , target_height(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , sigma_x(0.0)
    , sigma_y(0.0)
    , sigma_theta(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _robot_id_type;
  _robot_id_type robot_id;

   typedef double _target_height_type;
  _target_height_type target_height;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _theta_type;
  _theta_type theta;

   typedef double _sigma_x_type;
  _sigma_x_type sigma_x;

   typedef double _sigma_y_type;
  _sigma_y_type sigma_y;

   typedef double _sigma_theta_type;
  _sigma_theta_type sigma_theta;





  typedef boost::shared_ptr< ::dist_project::robot_data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dist_project::robot_data_<ContainerAllocator> const> ConstPtr;

}; // struct robot_data_

typedef ::dist_project::robot_data_<std::allocator<void> > robot_data;

typedef boost::shared_ptr< ::dist_project::robot_data > robot_dataPtr;
typedef boost::shared_ptr< ::dist_project::robot_data const> robot_dataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dist_project::robot_data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dist_project::robot_data_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dist_project::robot_data_<ContainerAllocator1> & lhs, const ::dist_project::robot_data_<ContainerAllocator2> & rhs)
{
  return lhs.robot_id == rhs.robot_id &&
    lhs.target_height == rhs.target_height &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta &&
    lhs.sigma_x == rhs.sigma_x &&
    lhs.sigma_y == rhs.sigma_y &&
    lhs.sigma_theta == rhs.sigma_theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dist_project::robot_data_<ContainerAllocator1> & lhs, const ::dist_project::robot_data_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dist_project

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dist_project::robot_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dist_project::robot_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dist_project::robot_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dist_project::robot_data_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dist_project::robot_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dist_project::robot_data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dist_project::robot_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "de74b7f6fa08eefd9305c86ead89bd1b";
  }

  static const char* value(const ::dist_project::robot_data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xde74b7f6fa08eefdULL;
  static const uint64_t static_value2 = 0x9305c86ead89bd1bULL;
};

template<class ContainerAllocator>
struct DataType< ::dist_project::robot_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dist_project/robot_data";
  }

  static const char* value(const ::dist_project::robot_data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dist_project::robot_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string robot_id\n"
"float64 target_height\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"float64 sigma_x\n"
"float64 sigma_y\n"
"float64 sigma_theta\n"
;
  }

  static const char* value(const ::dist_project::robot_data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dist_project::robot_data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_id);
      stream.next(m.target_height);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
      stream.next(m.sigma_x);
      stream.next(m.sigma_y);
      stream.next(m.sigma_theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct robot_data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dist_project::robot_data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dist_project::robot_data_<ContainerAllocator>& v)
  {
    s << indent << "robot_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.robot_id);
    s << indent << "target_height: ";
    Printer<double>::stream(s, indent + "  ", v.target_height);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
    s << indent << "sigma_x: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_x);
    s << indent << "sigma_y: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_y);
    s << indent << "sigma_theta: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIST_PROJECT_MESSAGE_ROBOT_DATA_H
