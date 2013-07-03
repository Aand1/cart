; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapEffectorTrajectoriesToObject-request.msg.html

(cl:defclass <MapEffectorTrajectoriesToObject-request> (roslisp-msg-protocol:ros-message)
  ((effector_traj
    :reader effector_traj
    :initarg :effector_traj
    :type manipulation_transforms-msg:EffectorTrajectories
    :initform (cl:make-instance 'manipulation_transforms-msg:EffectorTrajectories)))
)

(cl:defclass MapEffectorTrajectoriesToObject-request (<MapEffectorTrajectoriesToObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorTrajectoriesToObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorTrajectoriesToObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorTrajectoriesToObject-request> is deprecated: use manipulation_transforms-srv:MapEffectorTrajectoriesToObject-request instead.")))

(cl:ensure-generic-function 'effector_traj-val :lambda-list '(m))
(cl:defmethod effector_traj-val ((m <MapEffectorTrajectoriesToObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_traj-val is deprecated.  Use manipulation_transforms-srv:effector_traj instead.")
  (effector_traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorTrajectoriesToObject-request>) ostream)
  "Serializes a message object of type '<MapEffectorTrajectoriesToObject-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'effector_traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorTrajectoriesToObject-request>) istream)
  "Deserializes a message object of type '<MapEffectorTrajectoriesToObject-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'effector_traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorTrajectoriesToObject-request>)))
  "Returns string type for a service object of type '<MapEffectorTrajectoriesToObject-request>"
  "manipulation_transforms/MapEffectorTrajectoriesToObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTrajectoriesToObject-request)))
  "Returns string type for a service object of type 'MapEffectorTrajectoriesToObject-request"
  "manipulation_transforms/MapEffectorTrajectoriesToObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorTrajectoriesToObject-request>)))
  "Returns md5sum for a message object of type '<MapEffectorTrajectoriesToObject-request>"
  "15f599d66139397f83e4e0cee99f00bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorTrajectoriesToObject-request)))
  "Returns md5sum for a message object of type 'MapEffectorTrajectoriesToObject-request"
  "15f599d66139397f83e4e0cee99f00bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorTrajectoriesToObject-request>)))
  "Returns full string definition for message of type '<MapEffectorTrajectoriesToObject-request>"
  (cl:format cl:nil "~%EffectorTrajectories effector_traj~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectories~%# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorTrajectoriesToObject-request)))
  "Returns full string definition for message of type 'MapEffectorTrajectoriesToObject-request"
  (cl:format cl:nil "~%EffectorTrajectories effector_traj~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectories~%# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorTrajectoriesToObject-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'effector_traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorTrajectoriesToObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorTrajectoriesToObject-request
    (cl:cons ':effector_traj (effector_traj msg))
))
;//! \htmlinclude MapEffectorTrajectoriesToObject-response.msg.html

(cl:defclass <MapEffectorTrajectoriesToObject-response> (roslisp-msg-protocol:ros-message)
  ((object_poses
    :reader object_poses
    :initarg :object_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (object_twists
    :reader object_twists
    :initarg :object_twists
    :type (cl:vector geometry_msgs-msg:TwistStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TwistStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TwistStamped)))
   (error
    :reader error
    :initarg :error
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MapEffectorTrajectoriesToObject-response (<MapEffectorTrajectoriesToObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorTrajectoriesToObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorTrajectoriesToObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorTrajectoriesToObject-response> is deprecated: use manipulation_transforms-srv:MapEffectorTrajectoriesToObject-response instead.")))

(cl:ensure-generic-function 'object_poses-val :lambda-list '(m))
(cl:defmethod object_poses-val ((m <MapEffectorTrajectoriesToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_poses-val is deprecated.  Use manipulation_transforms-srv:object_poses instead.")
  (object_poses m))

(cl:ensure-generic-function 'object_twists-val :lambda-list '(m))
(cl:defmethod object_twists-val ((m <MapEffectorTrajectoriesToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_twists-val is deprecated.  Use manipulation_transforms-srv:object_twists instead.")
  (object_twists m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <MapEffectorTrajectoriesToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:error-val is deprecated.  Use manipulation_transforms-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorTrajectoriesToObject-response>) ostream)
  "Serializes a message object of type '<MapEffectorTrajectoriesToObject-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'object_poses))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object_twists))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'object_twists))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'error))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorTrajectoriesToObject-response>) istream)
  "Deserializes a message object of type '<MapEffectorTrajectoriesToObject-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object_twists) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object_twists)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:TwistStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'error) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'error)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorTrajectoriesToObject-response>)))
  "Returns string type for a service object of type '<MapEffectorTrajectoriesToObject-response>"
  "manipulation_transforms/MapEffectorTrajectoriesToObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTrajectoriesToObject-response)))
  "Returns string type for a service object of type 'MapEffectorTrajectoriesToObject-response"
  "manipulation_transforms/MapEffectorTrajectoriesToObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorTrajectoriesToObject-response>)))
  "Returns md5sum for a message object of type '<MapEffectorTrajectoriesToObject-response>"
  "15f599d66139397f83e4e0cee99f00bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorTrajectoriesToObject-response)))
  "Returns md5sum for a message object of type 'MapEffectorTrajectoriesToObject-response"
  "15f599d66139397f83e4e0cee99f00bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorTrajectoriesToObject-response>)))
  "Returns full string definition for message of type '<MapEffectorTrajectoriesToObject-response>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] object_poses~%geometry_msgs/TwistStamped[] object_twists~%~%~%float64[] error~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorTrajectoriesToObject-response)))
  "Returns full string definition for message of type 'MapEffectorTrajectoriesToObject-response"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] object_poses~%geometry_msgs/TwistStamped[] object_twists~%~%~%float64[] error~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorTrajectoriesToObject-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_twists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'error) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorTrajectoriesToObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorTrajectoriesToObject-response
    (cl:cons ':object_poses (object_poses msg))
    (cl:cons ':object_twists (object_twists msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapEffectorTrajectoriesToObject)))
  'MapEffectorTrajectoriesToObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapEffectorTrajectoriesToObject)))
  'MapEffectorTrajectoriesToObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTrajectoriesToObject)))
  "Returns string type for a service object of type '<MapEffectorTrajectoriesToObject>"
  "manipulation_transforms/MapEffectorTrajectoriesToObject")