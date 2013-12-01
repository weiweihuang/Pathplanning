/* Auto-generated by genmsg_cpp for file /home/huangwei/groovy_workspace/pathplanning/interactive_walk_planner_new/msg/task_mode.msg */
#ifndef INTERACTIVE_WALK_PLANNER_NEW_MESSAGE_TASK_MODE_H
#define INTERACTIVE_WALK_PLANNER_NEW_MESSAGE_TASK_MODE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace interactive_walk_planner_new
{
template <class ContainerAllocator>
struct task_mode_ {
  typedef task_mode_<ContainerAllocator> Type;

  task_mode_()
  : header()
  , x(0)
  {
  }

  task_mode_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _x_type;
  int32_t x;


  typedef boost::shared_ptr< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::interactive_walk_planner_new::task_mode_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct task_mode
typedef  ::interactive_walk_planner_new::task_mode_<std::allocator<void> > task_mode;

typedef boost::shared_ptr< ::interactive_walk_planner_new::task_mode> task_modePtr;
typedef boost::shared_ptr< ::interactive_walk_planner_new::task_mode const> task_modeConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::interactive_walk_planner_new::task_mode_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace interactive_walk_planner_new

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::interactive_walk_planner_new::task_mode_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a2b119ca6a13986b0b1e718a6981a87f";
  }

  static const char* value(const  ::interactive_walk_planner_new::task_mode_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa2b119ca6a13986bULL;
  static const uint64_t static_value2 = 0x0b1e718a6981a87fULL;
};

template<class ContainerAllocator>
struct DataType< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > {
  static const char* value() 
  {
    return "interactive_walk_planner_new/task_mode";
  }

  static const char* value(const  ::interactive_walk_planner_new::task_mode_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
int32 x\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::interactive_walk_planner_new::task_mode_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::interactive_walk_planner_new::task_mode_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct task_mode_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::interactive_walk_planner_new::task_mode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::interactive_walk_planner_new::task_mode_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
  }
};


} // namespace message_operations
} // namespace ros

#endif // INTERACTIVE_WALK_PLANNER_NEW_MESSAGE_TASK_MODE_H

