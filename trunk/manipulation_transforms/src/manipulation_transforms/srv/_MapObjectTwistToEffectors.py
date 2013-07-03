"""autogenerated by genpy from manipulation_transforms/MapObjectTwistToEffectorsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class MapObjectTwistToEffectorsRequest(genpy.Message):
  _md5sum = "520cb950a1ab872cff62b14155b5a20c"
  _type = "manipulation_transforms/MapObjectTwistToEffectorsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
geometry_msgs/TwistStamped object_twist

================================================================================
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['object_twist']
  _slot_types = ['geometry_msgs/TwistStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       object_twist

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MapObjectTwistToEffectorsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.object_twist is None:
        self.object_twist = geometry_msgs.msg.TwistStamped()
    else:
      self.object_twist = geometry_msgs.msg.TwistStamped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.object_twist.header.seq, _x.object_twist.header.stamp.secs, _x.object_twist.header.stamp.nsecs))
      _x = self.object_twist.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6d.pack(_x.object_twist.twist.linear.x, _x.object_twist.twist.linear.y, _x.object_twist.twist.linear.z, _x.object_twist.twist.angular.x, _x.object_twist.twist.angular.y, _x.object_twist.twist.angular.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.object_twist is None:
        self.object_twist = geometry_msgs.msg.TwistStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.object_twist.header.seq, _x.object_twist.header.stamp.secs, _x.object_twist.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object_twist.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.object_twist.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.object_twist.twist.linear.x, _x.object_twist.twist.linear.y, _x.object_twist.twist.linear.z, _x.object_twist.twist.angular.x, _x.object_twist.twist.angular.y, _x.object_twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.object_twist.header.seq, _x.object_twist.header.stamp.secs, _x.object_twist.header.stamp.nsecs))
      _x = self.object_twist.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6d.pack(_x.object_twist.twist.linear.x, _x.object_twist.twist.linear.y, _x.object_twist.twist.linear.z, _x.object_twist.twist.angular.x, _x.object_twist.twist.angular.y, _x.object_twist.twist.angular.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.object_twist is None:
        self.object_twist = geometry_msgs.msg.TwistStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.object_twist.header.seq, _x.object_twist.header.stamp.secs, _x.object_twist.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object_twist.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.object_twist.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.object_twist.twist.linear.x, _x.object_twist.twist.linear.y, _x.object_twist.twist.linear.z, _x.object_twist.twist.angular.x, _x.object_twist.twist.angular.y, _x.object_twist.twist.angular.z,) = _struct_6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_6d = struct.Struct("<6d")
"""autogenerated by genpy from manipulation_transforms/MapObjectTwistToEffectorsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class MapObjectTwistToEffectorsResponse(genpy.Message):
  _md5sum = "9e6592ec3797ef41e0f2bd288e07c910"
  _type = "manipulation_transforms/MapObjectTwistToEffectorsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
geometry_msgs/TwistStamped[] effector_twists



================================================================================
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into it's linear and angular parts. 
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['effector_twists']
  _slot_types = ['geometry_msgs/TwistStamped[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       effector_twists

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MapObjectTwistToEffectorsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.effector_twists is None:
        self.effector_twists = []
    else:
      self.effector_twists = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.effector_twists)
      buff.write(_struct_I.pack(length))
      for val1 in self.effector_twists:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.twist
        _v4 = _v3.linear
        _x = _v4
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = _v3.angular
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.effector_twists is None:
        self.effector_twists = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.effector_twists = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TwistStamped()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8')
        else:
          _v6.frame_id = str[start:end]
        _v8 = val1.twist
        _v9 = _v8.linear
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v10 = _v8.angular
        _x = _v10
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.effector_twists.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.effector_twists)
      buff.write(_struct_I.pack(length))
      for val1 in self.effector_twists:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v13 = val1.twist
        _v14 = _v13.linear
        _x = _v14
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v15 = _v13.angular
        _x = _v15
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.effector_twists is None:
        self.effector_twists = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.effector_twists = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.TwistStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8')
        else:
          _v16.frame_id = str[start:end]
        _v18 = val1.twist
        _v19 = _v18.linear
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v20 = _v18.angular
        _x = _v20
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.effector_twists.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
class MapObjectTwistToEffectors(object):
  _type          = 'manipulation_transforms/MapObjectTwistToEffectors'
  _md5sum = '1bb5479e5565270d07bd4dd38bdd26c9'
  _request_class  = MapObjectTwistToEffectorsRequest
  _response_class = MapObjectTwistToEffectorsResponse
