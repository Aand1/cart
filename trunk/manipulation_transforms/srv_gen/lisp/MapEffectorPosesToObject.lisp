; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapEffectorPosesToObject-request.msg.html

(cl:defclass <MapEffectorPosesToObject-request> (roslisp-msg-protocol:ros-message)
  ((effector_poses
    :reader effector_poses
    :initarg :effector_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass MapEffectorPosesToObject-request (<MapEffectorPosesToObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorPosesToObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorPosesToObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorPosesToObject-request> is deprecated: use manipulation_transforms-srv:MapEffectorPosesToObject-request instead.")))

(cl:ensure-generic-function 'effector_poses-val :lambda-list '(m))
(cl:defmethod effector_poses-val ((m <MapEffectorPosesToObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_poses-val is deprecated.  Use manipulation_transforms-srv:effector_poses instead.")
  (effector_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorPosesToObject-request>) ostream)
  "Serializes a message object of type '<MapEffectorPosesToObject-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'effector_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'effector_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorPosesToObject-request>) istream)
  "Deserializes a message object of type '<MapEffectorPosesToObject-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'effector_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'effector_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorPosesToObject-request>)))
  "Returns string type for a service object of type '<MapEffectorPosesToObject-request>"
  "manipulation_transforms/MapEffectorPosesToObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorPosesToObject-request)))
  "Returns string type for a service object of type 'MapEffectorPosesToObject-request"
  "manipulation_transforms/MapEffectorPosesToObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorPosesToObject-request>)))
  "Returns md5sum for a message object of type '<MapEffectorPosesToObject-request>"
  "5abafc6dc3670943f1fde6e363499078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorPosesToObject-request)))
  "Returns md5sum for a message object of type 'MapEffectorPosesToObject-request"
  "5abafc6dc3670943f1fde6e363499078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorPosesToObject-request>)))
  "Returns full string definition for message of type '<MapEffectorPosesToObject-request>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] effector_poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorPosesToObject-request)))
  "Returns full string definition for message of type 'MapEffectorPosesToObject-request"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] effector_poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorPosesToObject-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'effector_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorPosesToObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorPosesToObject-request
    (cl:cons ':effector_poses (effector_poses msg))
))
;//! \htmlinclude MapEffectorPosesToObject-response.msg.html

(cl:defclass <MapEffectorPosesToObject-response> (roslisp-msg-protocol:ros-message)
  ((object_pose
    :reader object_pose
    :initarg :object_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (error
    :reader error
    :initarg :error
    :type cl:float
    :initform 0.0))
)

(cl:defclass MapEffectorPosesToObject-response (<MapEffectorPosesToObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorPosesToObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorPosesToObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorPosesToObject-response> is deprecated: use manipulation_transforms-srv:MapEffectorPosesToObject-response instead.")))

(cl:ensure-generic-function 'object_pose-val :lambda-list '(m))
(cl:defmethod object_pose-val ((m <MapEffectorPosesToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_pose-val is deprecated.  Use manipulation_transforms-srv:object_pose instead.")
  (object_pose m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <MapEffectorPosesToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:error-val is deprecated.  Use manipulation_transforms-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorPosesToObject-response>) ostream)
  "Serializes a message object of type '<MapEffectorPosesToObject-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorPosesToObject-response>) istream)
  "Deserializes a message object of type '<MapEffectorPosesToObject-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorPosesToObject-response>)))
  "Returns string type for a service object of type '<MapEffectorPosesToObject-response>"
  "manipulation_transforms/MapEffectorPosesToObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorPosesToObject-response)))
  "Returns string type for a service object of type 'MapEffectorPosesToObject-response"
  "manipulation_transforms/MapEffectorPosesToObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorPosesToObject-response>)))
  "Returns md5sum for a message object of type '<MapEffectorPosesToObject-response>"
  "5abafc6dc3670943f1fde6e363499078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorPosesToObject-response)))
  "Returns md5sum for a message object of type 'MapEffectorPosesToObject-response"
  "5abafc6dc3670943f1fde6e363499078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorPosesToObject-response>)))
  "Returns full string definition for message of type '<MapEffectorPosesToObject-response>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped object_pose~%~%~%float64 error~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorPosesToObject-response)))
  "Returns full string definition for message of type 'MapEffectorPosesToObject-response"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped object_pose~%~%~%float64 error~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorPosesToObject-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_pose))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorPosesToObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorPosesToObject-response
    (cl:cons ':object_pose (object_pose msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapEffectorPosesToObject)))
  'MapEffectorPosesToObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapEffectorPosesToObject)))
  'MapEffectorPosesToObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorPosesToObject)))
  "Returns string type for a service object of type '<MapEffectorPosesToObject>"
  "manipulation_transforms/MapEffectorPosesToObject")