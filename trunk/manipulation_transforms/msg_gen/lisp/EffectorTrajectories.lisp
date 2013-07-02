; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-msg)


;//! \htmlinclude EffectorTrajectories.msg.html

(cl:defclass <EffectorTrajectories> (roslisp-msg-protocol:ros-message)
  ((pt
    :reader pt
    :initarg :pt
    :type (cl:vector manipulation_transforms-msg:EffectorTrajectoriesPoint)
   :initform (cl:make-array 0 :element-type 'manipulation_transforms-msg:EffectorTrajectoriesPoint :initial-element (cl:make-instance 'manipulation_transforms-msg:EffectorTrajectoriesPoint)))
   (names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass EffectorTrajectories (<EffectorTrajectories>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EffectorTrajectories>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EffectorTrajectories)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-msg:<EffectorTrajectories> is deprecated: use manipulation_transforms-msg:EffectorTrajectories instead.")))

(cl:ensure-generic-function 'pt-val :lambda-list '(m))
(cl:defmethod pt-val ((m <EffectorTrajectories>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-msg:pt-val is deprecated.  Use manipulation_transforms-msg:pt instead.")
  (pt m))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <EffectorTrajectories>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-msg:names-val is deprecated.  Use manipulation_transforms-msg:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EffectorTrajectories>) ostream)
  "Serializes a message object of type '<EffectorTrajectories>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pt))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EffectorTrajectories>) istream)
  "Deserializes a message object of type '<EffectorTrajectories>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pt) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pt)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'manipulation_transforms-msg:EffectorTrajectoriesPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EffectorTrajectories>)))
  "Returns string type for a message object of type '<EffectorTrajectories>"
  "manipulation_transforms/EffectorTrajectories")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EffectorTrajectories)))
  "Returns string type for a message object of type 'EffectorTrajectories"
  "manipulation_transforms/EffectorTrajectories")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EffectorTrajectories>)))
  "Returns md5sum for a message object of type '<EffectorTrajectories>"
  "bf27638dd852ff57ecedb67398cbfd2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EffectorTrajectories)))
  "Returns md5sum for a message object of type 'EffectorTrajectories"
  "bf27638dd852ff57ecedb67398cbfd2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EffectorTrajectories>)))
  "Returns full string definition for message of type '<EffectorTrajectories>"
  (cl:format cl:nil "# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EffectorTrajectories)))
  "Returns full string definition for message of type 'EffectorTrajectories"
  (cl:format cl:nil "# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EffectorTrajectories>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pt) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EffectorTrajectories>))
  "Converts a ROS message object to a list"
  (cl:list 'EffectorTrajectories
    (cl:cons ':pt (pt msg))
    (cl:cons ':names (names msg))
))
