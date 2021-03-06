"""autogenerated by genpy from sbpl_cart_planner/SBPLCartPlannerStats.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import cart_pushing_msgs.msg
import std_msgs.msg

class SBPLCartPlannerStats(genpy.Message):
  _md5sum = "59dcf49825e7e59499a38ffdd0daa999"
  _type = "sbpl_cart_planner/SBPLCartPlannerStats"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#planner stats
float64 initial_epsilon
float64 final_epsilon
bool plan_to_first_solution
float64 allocated_time
float64 actual_time
float64 time_to_first_solution
float64 solution_cost
float64 path_size
int64 final_number_of_expands
int64 number_of_expands_initial_solution

#problem stats
geometry_msgs/PoseStamped start
float64 start_cart_angle
geometry_msgs/PoseStamped goal
float64 goal_cart_angle

#solution
cart_pushing_msgs/RobotCartPath solution
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: cart_pushing_msgs/RobotCartPath
Header header
RobotCartConfiguration[] path
================================================================================
MSG: cart_pushing_msgs/RobotCartConfiguration
# Robot's pose in reference frame
geometry_msgs/Pose robot_pose

# Cart's pose in base frame
geometry_msgs/Pose cart_pose
"""
  __slots__ = ['initial_epsilon','final_epsilon','plan_to_first_solution','allocated_time','actual_time','time_to_first_solution','solution_cost','path_size','final_number_of_expands','number_of_expands_initial_solution','start','start_cart_angle','goal','goal_cart_angle','solution']
  _slot_types = ['float64','float64','bool','float64','float64','float64','float64','float64','int64','int64','geometry_msgs/PoseStamped','float64','geometry_msgs/PoseStamped','float64','cart_pushing_msgs/RobotCartPath']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       initial_epsilon,final_epsilon,plan_to_first_solution,allocated_time,actual_time,time_to_first_solution,solution_cost,path_size,final_number_of_expands,number_of_expands_initial_solution,start,start_cart_angle,goal,goal_cart_angle,solution

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SBPLCartPlannerStats, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.initial_epsilon is None:
        self.initial_epsilon = 0.
      if self.final_epsilon is None:
        self.final_epsilon = 0.
      if self.plan_to_first_solution is None:
        self.plan_to_first_solution = False
      if self.allocated_time is None:
        self.allocated_time = 0.
      if self.actual_time is None:
        self.actual_time = 0.
      if self.time_to_first_solution is None:
        self.time_to_first_solution = 0.
      if self.solution_cost is None:
        self.solution_cost = 0.
      if self.path_size is None:
        self.path_size = 0.
      if self.final_number_of_expands is None:
        self.final_number_of_expands = 0
      if self.number_of_expands_initial_solution is None:
        self.number_of_expands_initial_solution = 0
      if self.start is None:
        self.start = geometry_msgs.msg.PoseStamped()
      if self.start_cart_angle is None:
        self.start_cart_angle = 0.
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      if self.goal_cart_angle is None:
        self.goal_cart_angle = 0.
      if self.solution is None:
        self.solution = cart_pushing_msgs.msg.RobotCartPath()
    else:
      self.initial_epsilon = 0.
      self.final_epsilon = 0.
      self.plan_to_first_solution = False
      self.allocated_time = 0.
      self.actual_time = 0.
      self.time_to_first_solution = 0.
      self.solution_cost = 0.
      self.path_size = 0.
      self.final_number_of_expands = 0
      self.number_of_expands_initial_solution = 0
      self.start = geometry_msgs.msg.PoseStamped()
      self.start_cart_angle = 0.
      self.goal = geometry_msgs.msg.PoseStamped()
      self.goal_cart_angle = 0.
      self.solution = cart_pushing_msgs.msg.RobotCartPath()

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
      buff.write(_struct_2dB5d2q3I.pack(_x.initial_epsilon, _x.final_epsilon, _x.plan_to_first_solution, _x.allocated_time, _x.actual_time, _x.time_to_first_solution, _x.solution_cost, _x.path_size, _x.final_number_of_expands, _x.number_of_expands_initial_solution, _x.start.header.seq, _x.start.header.stamp.secs, _x.start.header.stamp.nsecs))
      _x = self.start.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d3I.pack(_x.start.pose.position.x, _x.start.pose.position.y, _x.start.pose.position.z, _x.start.pose.orientation.x, _x.start.pose.orientation.y, _x.start.pose.orientation.z, _x.start.pose.orientation.w, _x.start_cart_angle, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs))
      _x = self.goal.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d3I.pack(_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.goal_cart_angle, _x.solution.header.seq, _x.solution.header.stamp.secs, _x.solution.header.stamp.nsecs))
      _x = self.solution.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.solution.path)
      buff.write(_struct_I.pack(length))
      for val1 in self.solution.path:
        _v1 = val1.robot_pose
        _v2 = _v1.position
        _x = _v2
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v3 = _v1.orientation
        _x = _v3
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _v4 = val1.cart_pose
        _v5 = _v4.position
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = _v4.orientation
        _x = _v6
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.start is None:
        self.start = geometry_msgs.msg.PoseStamped()
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      if self.solution is None:
        self.solution = cart_pushing_msgs.msg.RobotCartPath()
      end = 0
      _x = self
      start = end
      end += 85
      (_x.initial_epsilon, _x.final_epsilon, _x.plan_to_first_solution, _x.allocated_time, _x.actual_time, _x.time_to_first_solution, _x.solution_cost, _x.path_size, _x.final_number_of_expands, _x.number_of_expands_initial_solution, _x.start.header.seq, _x.start.header.stamp.secs, _x.start.header.stamp.nsecs,) = _struct_2dB5d2q3I.unpack(str[start:end])
      self.plan_to_first_solution = bool(self.plan_to_first_solution)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.start.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 76
      (_x.start.pose.position.x, _x.start.pose.position.y, _x.start.pose.position.z, _x.start.pose.orientation.x, _x.start.pose.orientation.y, _x.start.pose.orientation.z, _x.start.pose.orientation.w, _x.start_cart_angle, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs,) = _struct_8d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.goal.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.goal.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 76
      (_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.goal_cart_angle, _x.solution.header.seq, _x.solution.header.stamp.secs, _x.solution.header.stamp.nsecs,) = _struct_8d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.solution.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.solution.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.solution.path = []
      for i in range(0, length):
        val1 = cart_pushing_msgs.msg.RobotCartConfiguration()
        _v7 = val1.robot_pose
        _v8 = _v7.position
        _x = _v8
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v9 = _v7.orientation
        _x = _v9
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _v10 = val1.cart_pose
        _v11 = _v10.position
        _x = _v11
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v12 = _v10.orientation
        _x = _v12
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.solution.path.append(val1)
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
      buff.write(_struct_2dB5d2q3I.pack(_x.initial_epsilon, _x.final_epsilon, _x.plan_to_first_solution, _x.allocated_time, _x.actual_time, _x.time_to_first_solution, _x.solution_cost, _x.path_size, _x.final_number_of_expands, _x.number_of_expands_initial_solution, _x.start.header.seq, _x.start.header.stamp.secs, _x.start.header.stamp.nsecs))
      _x = self.start.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d3I.pack(_x.start.pose.position.x, _x.start.pose.position.y, _x.start.pose.position.z, _x.start.pose.orientation.x, _x.start.pose.orientation.y, _x.start.pose.orientation.z, _x.start.pose.orientation.w, _x.start_cart_angle, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs))
      _x = self.goal.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d3I.pack(_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.goal_cart_angle, _x.solution.header.seq, _x.solution.header.stamp.secs, _x.solution.header.stamp.nsecs))
      _x = self.solution.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.solution.path)
      buff.write(_struct_I.pack(length))
      for val1 in self.solution.path:
        _v13 = val1.robot_pose
        _v14 = _v13.position
        _x = _v14
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v15 = _v13.orientation
        _x = _v15
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        _v16 = val1.cart_pose
        _v17 = _v16.position
        _x = _v17
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v18 = _v16.orientation
        _x = _v18
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.start is None:
        self.start = geometry_msgs.msg.PoseStamped()
      if self.goal is None:
        self.goal = geometry_msgs.msg.PoseStamped()
      if self.solution is None:
        self.solution = cart_pushing_msgs.msg.RobotCartPath()
      end = 0
      _x = self
      start = end
      end += 85
      (_x.initial_epsilon, _x.final_epsilon, _x.plan_to_first_solution, _x.allocated_time, _x.actual_time, _x.time_to_first_solution, _x.solution_cost, _x.path_size, _x.final_number_of_expands, _x.number_of_expands_initial_solution, _x.start.header.seq, _x.start.header.stamp.secs, _x.start.header.stamp.nsecs,) = _struct_2dB5d2q3I.unpack(str[start:end])
      self.plan_to_first_solution = bool(self.plan_to_first_solution)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.start.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 76
      (_x.start.pose.position.x, _x.start.pose.position.y, _x.start.pose.position.z, _x.start.pose.orientation.x, _x.start.pose.orientation.y, _x.start.pose.orientation.z, _x.start.pose.orientation.w, _x.start_cart_angle, _x.goal.header.seq, _x.goal.header.stamp.secs, _x.goal.header.stamp.nsecs,) = _struct_8d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.goal.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.goal.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 76
      (_x.goal.pose.position.x, _x.goal.pose.position.y, _x.goal.pose.position.z, _x.goal.pose.orientation.x, _x.goal.pose.orientation.y, _x.goal.pose.orientation.z, _x.goal.pose.orientation.w, _x.goal_cart_angle, _x.solution.header.seq, _x.solution.header.stamp.secs, _x.solution.header.stamp.nsecs,) = _struct_8d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.solution.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.solution.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.solution.path = []
      for i in range(0, length):
        val1 = cart_pushing_msgs.msg.RobotCartConfiguration()
        _v19 = val1.robot_pose
        _v20 = _v19.position
        _x = _v20
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v21 = _v19.orientation
        _x = _v21
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        _v22 = val1.cart_pose
        _v23 = _v22.position
        _x = _v23
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v24 = _v22.orientation
        _x = _v24
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.solution.path.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_8d3I = struct.Struct("<8d3I")
_struct_4d = struct.Struct("<4d")
_struct_2dB5d2q3I = struct.Struct("<2dB5d2q3I")
_struct_3d = struct.Struct("<3d")
