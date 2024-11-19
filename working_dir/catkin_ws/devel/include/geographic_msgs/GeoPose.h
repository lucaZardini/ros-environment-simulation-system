// Generated by gencpp from file geographic_msgs/GeoPose.msg
// DO NOT EDIT!


#ifndef GEOGRAPHIC_MSGS_MESSAGE_GEOPOSE_H
#define GEOGRAPHIC_MSGS_MESSAGE_GEOPOSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Quaternion.h>

namespace geographic_msgs
{
template <class ContainerAllocator>
struct GeoPose_
{
  typedef GeoPose_<ContainerAllocator> Type;

  GeoPose_()
    : position()
    , orientation()  {
    }
  GeoPose_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , orientation(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geographic_msgs::GeoPoint_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;





  typedef boost::shared_ptr< ::geographic_msgs::GeoPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geographic_msgs::GeoPose_<ContainerAllocator> const> ConstPtr;

}; // struct GeoPose_

typedef ::geographic_msgs::GeoPose_<std::allocator<void> > GeoPose;

typedef boost::shared_ptr< ::geographic_msgs::GeoPose > GeoPosePtr;
typedef boost::shared_ptr< ::geographic_msgs::GeoPose const> GeoPoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geographic_msgs::GeoPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::geographic_msgs::GeoPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geographic_msgs::GeoPose_<ContainerAllocator1> & lhs, const ::geographic_msgs::GeoPose_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.orientation == rhs.orientation;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geographic_msgs::GeoPose_<ContainerAllocator1> & lhs, const ::geographic_msgs::GeoPose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geographic_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::geographic_msgs::GeoPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::geographic_msgs::GeoPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geographic_msgs::GeoPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geographic_msgs::GeoPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geographic_msgs::GeoPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::geographic_msgs::GeoPose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geographic_msgs::GeoPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "778680b5172de58b7c057d973576c784";
  }

  static const char* value(const ::geographic_msgs::GeoPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x778680b5172de58bULL;
  static const uint64_t static_value2 = 0x7c057d973576c784ULL;
};

template<class ContainerAllocator>
struct DataType< ::geographic_msgs::GeoPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geographic_msgs/GeoPose";
  }

  static const char* value(const ::geographic_msgs::GeoPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geographic_msgs::GeoPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Geographic pose, using the WGS 84 reference ellipsoid.\n"
"#\n"
"# Orientation uses the East-North-Up (ENU) frame of reference.\n"
"# (But, what about singularities at the poles?)\n"
"\n"
"GeoPoint position\n"
"geometry_msgs/Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geographic_msgs/GeoPoint\n"
"# Geographic point, using the WGS 84 reference ellipsoid.\n"
"\n"
"# Latitude [degrees]. Positive is north of equator; negative is south\n"
"# (-90 <= latitude <= +90).\n"
"float64 latitude\n"
"\n"
"# Longitude [degrees]. Positive is east of prime meridian; negative is\n"
"# west (-180 <= longitude <= +180). At the poles, latitude is -90 or\n"
"# +90, and longitude is irrelevant, but must be in range.\n"
"float64 longitude\n"
"\n"
"# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).\n"
"float64 altitude\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::geographic_msgs::GeoPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geographic_msgs::GeoPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GeoPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geographic_msgs::GeoPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geographic_msgs::GeoPose_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geographic_msgs::GeoPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GEOGRAPHIC_MSGS_MESSAGE_GEOPOSE_H
