/* Auto-generated by genmsg_cpp for file /home/diro/ros_workspace/final_project/trunk/manipulation_transforms/srv/MapEffectorPosesToObject.srv */
#ifndef MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORPOSESTOOBJECT_H
#define MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORPOSESTOOBJECT_H
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
struct MapEffectorPosesToObjectRequest_ {
  typedef MapEffectorPosesToObjectRequest_<ContainerAllocator> Type;

  MapEffectorPosesToObjectRequest_()
  : effector_poses()
  {
  }

  MapEffectorPosesToObjectRequest_(const ContainerAllocator& _alloc)
  : effector_poses(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _effector_poses_type;
  std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  effector_poses;


  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MapEffectorPosesToObjectRequest
typedef  ::manipulation_transforms::MapEffectorPosesToObjectRequest_<std::allocator<void> > MapEffectorPosesToObjectRequest;

typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectRequest> MapEffectorPosesToObjectRequestPtr;
typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectRequest const> MapEffectorPosesToObjectRequestConstPtr;


template <class ContainerAllocator>
struct MapEffectorPosesToObjectResponse_ {
  typedef MapEffectorPosesToObjectResponse_<ContainerAllocator> Type;

  MapEffectorPosesToObjectResponse_()
  : object_pose()
  , error(0.0)
  {
  }

  MapEffectorPosesToObjectResponse_(const ContainerAllocator& _alloc)
  : object_pose(_alloc)
  , error(0.0)
  {
  }

  typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _object_pose_type;
   ::geometry_msgs::PoseStamped_<ContainerAllocator>  object_pose;

  typedef double _error_type;
  double error;


  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MapEffectorPosesToObjectResponse
typedef  ::manipulation_transforms::MapEffectorPosesToObjectResponse_<std::allocator<void> > MapEffectorPosesToObjectResponse;

typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectResponse> MapEffectorPosesToObjectResponsePtr;
typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorPosesToObjectResponse const> MapEffectorPosesToObjectResponseConstPtr;

struct MapEffectorPosesToObject
{

typedef MapEffectorPosesToObjectRequest Request;
typedef MapEffectorPosesToObjectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct MapEffectorPosesToObject
} // namespace manipulation_transforms

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "175b382c67666ecb6c87e1a22c5729fc";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x175b382c67666ecbULL;
  static const uint64_t static_value2 = 0x6c87e1a22c5729fcULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorPosesToObjectRequest";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
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

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bed46a04d25249170924e13bfa1fe9a4";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbed46a04d2524917ULL;
  static const uint64_t static_value2 = 0x0924e13bfa1fe9a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorPosesToObjectResponse";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
geometry_msgs/PoseStamped object_pose\n\
\n\
\n\
float64 error\n\
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
";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.effector_poses);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MapEffectorPosesToObjectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.object_pose);
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MapEffectorPosesToObjectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<manipulation_transforms::MapEffectorPosesToObject> {
  static const char* value() 
  {
    return "5abafc6dc3670943f1fde6e363499078";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObject&) { return value(); } 
};

template<>
struct DataType<manipulation_transforms::MapEffectorPosesToObject> {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorPosesToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObject&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5abafc6dc3670943f1fde6e363499078";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorPosesToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5abafc6dc3670943f1fde6e363499078";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorPosesToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorPosesToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORPOSESTOOBJECT_H
