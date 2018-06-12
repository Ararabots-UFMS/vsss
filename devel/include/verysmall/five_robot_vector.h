// Generated by gencpp from file verysmall/five_robot_vector.msg
// DO NOT EDIT!


#ifndef VERYSMALL_MESSAGE_FIVE_ROBOT_VECTOR_H
#define VERYSMALL_MESSAGE_FIVE_ROBOT_VECTOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace verysmall
{
template <class ContainerAllocator>
struct five_robot_vector_
{
  typedef five_robot_vector_<ContainerAllocator> Type;

  five_robot_vector_()
    : robot_angle_vector_1(0.0)
    , robot_angle_vector_2(0.0)
    , robot_angle_vector_3(0.0)
    , robot_angle_vector_4(0.0)
    , robot_angle_vector_5(0.0)  {
    }
  five_robot_vector_(const ContainerAllocator& _alloc)
    : robot_angle_vector_1(0.0)
    , robot_angle_vector_2(0.0)
    , robot_angle_vector_3(0.0)
    , robot_angle_vector_4(0.0)
    , robot_angle_vector_5(0.0)  {
  (void)_alloc;
    }



   typedef double _robot_angle_vector_1_type;
  _robot_angle_vector_1_type robot_angle_vector_1;

   typedef double _robot_angle_vector_2_type;
  _robot_angle_vector_2_type robot_angle_vector_2;

   typedef double _robot_angle_vector_3_type;
  _robot_angle_vector_3_type robot_angle_vector_3;

   typedef double _robot_angle_vector_4_type;
  _robot_angle_vector_4_type robot_angle_vector_4;

   typedef double _robot_angle_vector_5_type;
  _robot_angle_vector_5_type robot_angle_vector_5;





  typedef boost::shared_ptr< ::verysmall::five_robot_vector_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::verysmall::five_robot_vector_<ContainerAllocator> const> ConstPtr;

}; // struct five_robot_vector_

typedef ::verysmall::five_robot_vector_<std::allocator<void> > five_robot_vector;

typedef boost::shared_ptr< ::verysmall::five_robot_vector > five_robot_vectorPtr;
typedef boost::shared_ptr< ::verysmall::five_robot_vector const> five_robot_vectorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::verysmall::five_robot_vector_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::verysmall::five_robot_vector_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace verysmall

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'verysmall': ['/home/marquesman/ararabots/src/verysmall/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::verysmall::five_robot_vector_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::verysmall::five_robot_vector_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::verysmall::five_robot_vector_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::verysmall::five_robot_vector_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::verysmall::five_robot_vector_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::verysmall::five_robot_vector_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::verysmall::five_robot_vector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "46d2ecd06a68bed7c3b95711f5808fee";
  }

  static const char* value(const ::verysmall::five_robot_vector_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x46d2ecd06a68bed7ULL;
  static const uint64_t static_value2 = 0xc3b95711f5808feeULL;
};

template<class ContainerAllocator>
struct DataType< ::verysmall::five_robot_vector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "verysmall/five_robot_vector";
  }

  static const char* value(const ::verysmall::five_robot_vector_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::verysmall::five_robot_vector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64   robot_angle_vector_1\n\
float64   robot_angle_vector_2\n\
float64   robot_angle_vector_3\n\
float64   robot_angle_vector_4\n\
float64   robot_angle_vector_5\n\
";
  }

  static const char* value(const ::verysmall::five_robot_vector_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::verysmall::five_robot_vector_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_angle_vector_1);
      stream.next(m.robot_angle_vector_2);
      stream.next(m.robot_angle_vector_3);
      stream.next(m.robot_angle_vector_4);
      stream.next(m.robot_angle_vector_5);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct five_robot_vector_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::verysmall::five_robot_vector_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::verysmall::five_robot_vector_<ContainerAllocator>& v)
  {
    s << indent << "robot_angle_vector_1: ";
    Printer<double>::stream(s, indent + "  ", v.robot_angle_vector_1);
    s << indent << "robot_angle_vector_2: ";
    Printer<double>::stream(s, indent + "  ", v.robot_angle_vector_2);
    s << indent << "robot_angle_vector_3: ";
    Printer<double>::stream(s, indent + "  ", v.robot_angle_vector_3);
    s << indent << "robot_angle_vector_4: ";
    Printer<double>::stream(s, indent + "  ", v.robot_angle_vector_4);
    s << indent << "robot_angle_vector_5: ";
    Printer<double>::stream(s, indent + "  ", v.robot_angle_vector_5);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VERYSMALL_MESSAGE_FIVE_ROBOT_VECTOR_H
