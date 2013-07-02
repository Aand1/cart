/* Auto-generated by genmsg_cpp for file /home/diro/ros_workspace/final_project/trunk/manipulation_transforms/srv/MapEffectorTwistsToObject.srv */
#ifndef MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORTWISTSTOOBJECT_H
#define MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORTWISTSTOOBJECT_H
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

#include "geometry_msgs/TwistStamped.h"


#include "geometry_msgs/TwistStamped.h"

namespace manipulation_transforms
{
template <class ContainerAllocator>
struct MapEffectorTwistsToObjectRequest_ {
  typedef MapEffectorTwistsToObjectRequest_<ContainerAllocator> Type;

  MapEffectorTwistsToObjectRequest_()
  : effector_twists()
  {
  }

  MapEffectorTwistsToObjectRequest_(const ContainerAllocator& _alloc)
  : effector_twists(_alloc)
  {
  }

  typedef std::vector< ::geometry_msgs::TwistStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::other >  _effector_twists_type;
  std::vector< ::geometry_msgs::TwistStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::other >  effector_twists;


  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MapEffectorTwistsToObjectRequest
typedef  ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<std::allocator<void> > MapEffectorTwistsToObjectRequest;

typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectRequest> MapEffectorTwistsToObjectRequestPtr;
typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectRequest const> MapEffectorTwistsToObjectRequestConstPtr;


template <class ContainerAllocator>
struct MapEffectorTwistsToObjectResponse_ {
  typedef MapEffectorTwistsToObjectResponse_<ContainerAllocator> Type;

  MapEffectorTwistsToObjectResponse_()
  : object_twist()
  , error(0.0)
  {
  }

  MapEffectorTwistsToObjectResponse_(const ContainerAllocator& _alloc)
  : object_twist(_alloc)
  , error(0.0)
  {
  }

  typedef  ::geometry_msgs::TwistStamped_<ContainerAllocator>  _object_twist_type;
   ::geometry_msgs::TwistStamped_<ContainerAllocator>  object_twist;

  typedef double _error_type;
  double error;


  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MapEffectorTwistsToObjectResponse
typedef  ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<std::allocator<void> > MapEffectorTwistsToObjectResponse;

typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectResponse> MapEffectorTwistsToObjectResponsePtr;
typedef boost::shared_ptr< ::manipulation_transforms::MapEffectorTwistsToObjectResponse const> MapEffectorTwistsToObjectResponseConstPtr;

struct MapEffectorTwistsToObject
{

typedef MapEffectorTwistsToObjectRequest Request;
typedef MapEffectorTwistsToObjectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct MapEffectorTwistsToObject
} // namespace manipulation_transforms

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9e6592ec3797ef41e0f2bd288e07c910";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9e6592ec3797ef41ULL;
  static const uint64_t static_value2 = 0xe0f2bd288e07c910ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorTwistsToObjectRequest";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
geometry_msgs/TwistStamped[] effector_twists\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistStamped\n\
# A twist with reference coordinate frame and timestamp\n\
Header header\n\
Twist twist\n\
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

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4ade889a0856a8f27d42bb0179bcdb9d";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4ade889a0856a8f2ULL;
  static const uint64_t static_value2 = 0x7d42bb0179bcdb9dULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorTwistsToObjectResponse";
  }

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
geometry_msgs/TwistStamped object_twist\n\
\n\
\n\
float64 error\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistStamped\n\
# A twist with reference coordinate frame and timestamp\n\
Header header\n\
Twist twist\n\
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

  static const char* value(const  ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.effector_twists);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MapEffectorTwistsToObjectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.object_twist);
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MapEffectorTwistsToObjectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<manipulation_transforms::MapEffectorTwistsToObject> {
  static const char* value() 
  {
    return "b562f5125b32085904e28b16f9470873";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObject&) { return value(); } 
};

template<>
struct DataType<manipulation_transforms::MapEffectorTwistsToObject> {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorTwistsToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObject&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b562f5125b32085904e28b16f9470873";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorTwistsToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObjectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b562f5125b32085904e28b16f9470873";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "manipulation_transforms/MapEffectorTwistsToObject";
  }

  static const char* value(const manipulation_transforms::MapEffectorTwistsToObjectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MANIPULATION_TRANSFORMS_SERVICE_MAPEFFECTORTWISTSTOOBJECT_H
