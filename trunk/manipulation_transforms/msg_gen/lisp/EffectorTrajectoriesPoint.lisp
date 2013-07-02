; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-msg)


;//! \htmlinclude EffectorTrajectoriesPoint.msg.html

(cl:defclass <EffectorTrajectoriesPoint> (roslisp-msg-protocol:ros-message)
  ((poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (twists
    :reader twists
    :initarg :twists
    :type (cl:vector geometry_msgs-msg:TwistStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TwistStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TwistStamped))))
)

(cl:defclass EffectorTrajectoriesPoint (<EffectorTrajectoriesPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EffectorTrajectoriesPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EffectorTrajectoriesPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-msg:<EffectorTrajectoriesPoint> is deprecated: use manipulation_transforms-msg:EffectorTrajectoriesPoint instead.")))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <EffectorTrajectoriesPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-msg:poses-val is deprecated.  Use manipulation_transforms-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'twists-val :lambda-list '(m))
(cl:defmethod twists-val ((m <EffectorTrajectoriesPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-msg:twists-val is deprecated.  Use manipulation_transforms-msg:twists instead.")
  (twists m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EffectorTrajectoriesPoint>) ostream)
  "Serializes a message object of type '<EffectorTrajectoriesPoint>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'twists))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'twists))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EffectorTrajectoriesPoint>) istream)
  "Deserializes a message object of type '<EffectorTrajectoriesPoint>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'twists) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'twists)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:TwistStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EffectorTrajectoriesPoint>)))
  "Returns string type for a message object of type '<EffectorTrajectoriesPoint>"
  "manipulation_transforms/EffectorTrajectoriesPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EffectorTrajectoriesPoint)))
  "Returns string type for a message object of type 'EffectorTrajectoriesPoint"
  "manipulation_transforms/EffectorTrajectoriesPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EffectorTrajectoriesPoint>)))
  "Returns md5sum for a message object of type '<EffectorTrajectoriesPoint>"
  "e801e3ee2cea9c4100e0018babf72f59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EffectorTrajectoriesPoint)))
  "Returns md5sum for a message object of type 'EffectorTrajectoriesPoint"
  "e801e3ee2cea9c4100e0018babf72f59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EffectorTrajectoriesPoint>)))
  "Returns full string definition for message of type '<EffectorTrajectoriesPoint>"
  (cl:format cl:nil "# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EffectorTrajectoriesPoint)))
  "Returns full string definition for message of type 'EffectorTrajectoriesPoint"
  (cl:format cl:nil "# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EffectorTrajectoriesPoint>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'twists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EffectorTrajectoriesPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'EffectorTrajectoriesPoint
    (cl:cons ':poses (poses msg))
    (cl:cons ':twists (twists msg))
))
