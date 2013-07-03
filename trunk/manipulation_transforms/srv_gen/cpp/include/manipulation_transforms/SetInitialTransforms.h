/* Auto-generated by genmsg_cpp for file /home/diro/ros_workspace/final_project/trunk/manipulation_transforms/srv/SetInitialTransforms.srv */
#ifndef MANIPULATION_TRANSFORMS_SERVICE_SETINITIALTRANSFORMS_H
#define MANIPULATION_TRANSFORMS_SERVICE_SETINITIALTRANSFORMS_H
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

#include "ros/service_traits.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"



namespace manipulation_transforms
{
template <class ContainerAllocator>
struct SetInitialTransformsRequest_ {
  typedef SetInitialTransformsRequest_<ContainerAllocator> Type;

  SetInitialTransformsRequest_()
  : object_pose()
  , effector_poses()
  {
  }

  SetInitialTransformsRequest_(const ContainerAllocator& _alloc)
  : object_pose(_alloc)
  , effector_poses(_alloc)
  {
  }

  typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _object_pose_type;
   ::geometry_msgs::PoseStamped_<ContainerAllocator>  object_pose;

  typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _effector_poses_type;
  std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  effector_poses;


  typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetInitialTransformsRequest
typedef  ::manipulation_transforms::SetInitialTransformsRequest_<std::allocator<void> > SetInitialTransformsRequest;

typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsRequest> SetInitialTransformsRequestPtr;
typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsRequest const> SetInitialTransformsRequestConstPtr;


template <class ContainerAllocator>
struct SetInitialTransformsResponse_ {
  typedef SetInitialTransformsResponse_<ContainerAllocator> Type;

  SetInitialTransformsResponse_()
  : success(false)
  {
  }

  SetInitialTransformsResponse_(const ContainerAllocator& _alloc)
  : success(false)
  {
  }

  typedef uint8_t _success_type;
  uint8_t success;


  typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetInitialTransformsResponse
typedef  ::manipulation_transforms::SetInitialTransformsResponse_<std::allocator<void> > SetInitialTransformsResponse;

typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsResponse> SetInitialTransformsResponsePtr;
typedef boost::shared_ptr< ::manipulation_transforms::SetInitialTransformsResponse const> SetInitialTransformsResponseConstPtr;

struct SetInitialTransforms
{

typedef SetInitialTransformsRequest Request;
typedef SetInitialTransformsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetInitialTransforms
} // namespace manipulation_transforms

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "20e1e914372addbf166e586475963be6";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x20e1e914372addbfULL;
  static const uint64_t static_value2 = 0x166e586475963be6ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/SetInitialTransformsRequest";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/PoseStamped object_pose\n\
geometry_msgs/PoseStamped[] effector_poses\n\
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
";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/SetInitialTransformsResponse";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
bool success\n\
\n\
\n\
";
  }

  static const char* value(const  ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.object_pose);
    stream.next(m.effector_poses);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetInitialTransformsRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.success);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetInitialTransformsResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<manipulation_transforms::SetInitialTransforms> {
  static const char* value() 
  {
    return "c6f24d628001d50710ead5449e25726f";
  }

  static const char* value(const manipulation_transforms::SetInitialTransforms&) { return value(); } 
};

template<>
struct DataType<manipulation_transforms::SetInitialTransforms> {
  static const char* value() 
  {
    return "manipulation_transforms/SetInitialTransforms";
  }

  static const char* value(const manipulation_transforms::SetInitialTransforms&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c6f24d628001d50710ead5449e25726f";
  }

  static const char* value(const manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/SetInitialTransforms";
  }

  static const char* value(const manipulation_transforms::SetInitialTransformsRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c6f24d628001d50710ead5449e25726f";
  }

  static const char* value(const manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/SetInitialTransforms";
  }

  static const char* value(const manipulation_transforms::SetInitialTransformsResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MANIPULATION_TRANSFORMS_SERVICE_SETINITIALTRANSFORMS_H

