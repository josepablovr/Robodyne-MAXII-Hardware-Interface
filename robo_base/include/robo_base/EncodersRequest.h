// Generated by gencpp from file xbot/EncodersRequest.msg
// DO NOT EDIT!


#ifndef XBOT_MESSAGE_ENCODERSREQUEST_H
#define XBOT_MESSAGE_ENCODERSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xbot
{
template <class ContainerAllocator>
struct EncodersRequest_
{
  typedef EncodersRequest_<ContainerAllocator> Type;

  EncodersRequest_()
    : a()
    , b()  {
    }
  EncodersRequest_(const ContainerAllocator& _alloc)
    : a(_alloc)
    , b(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _a_type;
  _a_type a;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _b_type;
  _b_type b;




  typedef boost::shared_ptr< ::xbot::EncodersRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xbot::EncodersRequest_<ContainerAllocator> const> ConstPtr;

}; // struct EncodersRequest_

typedef ::xbot::EncodersRequest_<std::allocator<void> > EncodersRequest;

typedef boost::shared_ptr< ::xbot::EncodersRequest > EncodersRequestPtr;
typedef boost::shared_ptr< ::xbot::EncodersRequest const> EncodersRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xbot::EncodersRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xbot::EncodersRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace xbot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::xbot::EncodersRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xbot::EncodersRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xbot::EncodersRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xbot::EncodersRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xbot::EncodersRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xbot::EncodersRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xbot::EncodersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "efc98bdd092d58ce0c705b4bd69d178c";
  }

  static const char* value(const ::xbot::EncodersRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xefc98bdd092d58ceULL;
  static const uint64_t static_value2 = 0x0c705b4bd69d178cULL;
};

template<class ContainerAllocator>
struct DataType< ::xbot::EncodersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xbot/EncodersRequest";
  }

  static const char* value(const ::xbot::EncodersRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xbot::EncodersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string a\n\
string b\n\
";
  }

  static const char* value(const ::xbot::EncodersRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xbot::EncodersRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct EncodersRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xbot::EncodersRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xbot::EncodersRequest_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XBOT_MESSAGE_ENCODERSREQUEST_H
