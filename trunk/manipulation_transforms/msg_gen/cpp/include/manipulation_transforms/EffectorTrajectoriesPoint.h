/* Auto-generated by genmsg_cpp for file /home/diro/ros_workspace/final_project/trunk/manipulation_transforms/msg/EffectorTrajectoriesPoint.msg */
#ifndef MANIPULATION_TRANSFORMS_MESSAGE_EFFECTORTRAJECTORIESPOINT_H
#define MANIPULATION_TRANSFORMS_MESSAGE_EFFECTORTRAJECTORIESPOINT_H
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

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

namespace manipulation_transforms
{
template <class ContainerAllocator>
struct EffectorTrajectoriesPoint_ {
  typedef EffectorTrajectoriesPoint_<ContainerAllocator> Type;

  EffectorTrajectoriesPoint_()
  : poses()
  , twists()
  {
  }

  EffectorTrajectoriesPoint_(const ContainerAllocator& _alloc)
  : poses(_alloc)
  , twists(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _poses_type;
  std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  poses;

  typedef std::vector< ::geometry_msgs::TwistStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::other >  _twists_type;
  std::vector< ::geometry_msgs::TwistStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::other >  twists;


  typedef boost::shared_ptr< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EffectorTrajectoriesPoint
typedef  ::manipulation_transforms::EffectorTrajectoriesPoint_<std::allocator<void> > EffectorTrajectoriesPoint;

typedef boost::shared_ptr< ::manipulation_transforms::EffectorTrajectoriesPoint> EffectorTrajectoriesPointPtr;
typedef boost::shared_ptr< ::manipulation_transforms::EffectorTrajectoriesPoint const> EffectorTrajectoriesPointConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace manipulation_transforms

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e801e3ee2cea9c4100e0018babf72f59";
  }

  static const char* value(const  ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe801e3ee2cea9c41ULL;
  static const uint64_t static_value2 = 0x00e0018babf72f59ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/EffectorTrajectoriesPoint";
  }

  static const char* value(const  ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Array of poses, one for each effector\n\
geometry_msgs/PoseStamped[] poses\n\
\n\
# Array of twists, one for each effector\n\
geometry_msgs/TwistStamped[] twists\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
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
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistStamped\n\
# A twist with reference coordinate frame and timestamp\n\
Header header\n\
Twist twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.poses);
    stream.next(m.twists);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EffectorTrajectoriesPoint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::manipulation_transforms::EffectorTrajectoriesPoint_<ContainerAllocator> & v) 
  {
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
    s << indent << "twists[]" << std::endl;
    for (size_t i = 0; i < v.twists.size(); ++i)
    {
      s << indent << "  twists[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.twists[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_TRANSFORMS_MESSAGE_EFFECTORTRAJECTORIESPOINT_H
