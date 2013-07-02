; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapEffectorTwistsToObject-request.msg.html

(cl:defclass <MapEffectorTwistsToObject-request> (roslisp-msg-protocol:ros-message)
  ((effector_twists
    :reader effector_twists
    :initarg :effector_twists
    :type (cl:vector geometry_msgs-msg:TwistStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TwistStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TwistStamped))))
)

(cl:defclass MapEffectorTwistsToObject-request (<MapEffectorTwistsToObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorTwistsToObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorTwistsToObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorTwistsToObject-request> is deprecated: use manipulation_transforms-srv:MapEffectorTwistsToObject-request instead.")))

(cl:ensure-generic-function 'effector_twists-val :lambda-list '(m))
(cl:defmethod effector_twists-val ((m <MapEffectorTwistsToObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_twists-val is deprecated.  Use manipulation_transforms-srv:effector_twists instead.")
  (effector_twists m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorTwistsToObject-request>) ostream)
  "Serializes a message object of type '<MapEffectorTwistsToObject-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'effector_twists))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'effector_twists))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorTwistsToObject-request>) istream)
  "Deserializes a message object of type '<MapEffectorTwistsToObject-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'effector_twists) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'effector_twists)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:TwistStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorTwistsToObject-request>)))
  "Returns string type for a service object of type '<MapEffectorTwistsToObject-request>"
  "manipulation_transforms/MapEffectorTwistsToObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTwistsToObject-request)))
  "Returns string type for a service object of type 'MapEffectorTwistsToObject-request"
  "manipulation_transforms/MapEffectorTwistsToObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorTwistsToObject-request>)))
  "Returns md5sum for a message object of type '<MapEffectorTwistsToObject-request>"
  "b562f5125b32085904e28b16f9470873")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorTwistsToObject-request)))
  "Returns md5sum for a message object of type 'MapEffectorTwistsToObject-request"
  "b562f5125b32085904e28b16f9470873")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorTwistsToObject-request>)))
  "Returns full string definition for message of type '<MapEffectorTwistsToObject-request>"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped[] effector_twists~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorTwistsToObject-request)))
  "Returns full string definition for message of type 'MapEffectorTwistsToObject-request"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped[] effector_twists~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorTwistsToObject-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'effector_twists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorTwistsToObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorTwistsToObject-request
    (cl:cons ':effector_twists (effector_twists msg))
))
;//! \htmlinclude MapEffectorTwistsToObject-response.msg.html

(cl:defclass <MapEffectorTwistsToObject-response> (roslisp-msg-protocol:ros-message)
  ((object_twist
    :reader object_twist
    :initarg :object_twist
    :type geometry_msgs-msg:TwistStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TwistStamped))
   (error
    :reader error
    :initarg :error
    :type cl:float
    :initform 0.0))
)

(cl:defclass MapEffectorTwistsToObject-response (<MapEffectorTwistsToObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapEffectorTwistsToObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapEffectorTwistsToObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapEffectorTwistsToObject-response> is deprecated: use manipulation_transforms-srv:MapEffectorTwistsToObject-response instead.")))

(cl:ensure-generic-function 'object_twist-val :lambda-list '(m))
(cl:defmethod object_twist-val ((m <MapEffectorTwistsToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_twist-val is deprecated.  Use manipulation_transforms-srv:object_twist instead.")
  (object_twist m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <MapEffectorTwistsToObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:error-val is deprecated.  Use manipulation_transforms-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapEffectorTwistsToObject-response>) ostream)
  "Serializes a message object of type '<MapEffectorTwistsToObject-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_twist) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapEffectorTwistsToObject-response>) istream)
  "Deserializes a message object of type '<MapEffectorTwistsToObject-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_twist) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapEffectorTwistsToObject-response>)))
  "Returns string type for a service object of type '<MapEffectorTwistsToObject-response>"
  "manipulation_transforms/MapEffectorTwistsToObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTwistsToObject-response)))
  "Returns string type for a service object of type 'MapEffectorTwistsToObject-response"
  "manipulation_transforms/MapEffectorTwistsToObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapEffectorTwistsToObject-response>)))
  "Returns md5sum for a message object of type '<MapEffectorTwistsToObject-response>"
  "b562f5125b32085904e28b16f9470873")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapEffectorTwistsToObject-response)))
  "Returns md5sum for a message object of type 'MapEffectorTwistsToObject-response"
  "b562f5125b32085904e28b16f9470873")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapEffectorTwistsToObject-response>)))
  "Returns full string definition for message of type '<MapEffectorTwistsToObject-response>"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped object_twist~%~%~%float64 error~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapEffectorTwistsToObject-response)))
  "Returns full string definition for message of type 'MapEffectorTwistsToObject-response"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped object_twist~%~%~%float64 error~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapEffectorTwistsToObject-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_twist))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapEffectorTwistsToObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapEffectorTwistsToObject-response
    (cl:cons ':object_twist (object_twist msg))
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapEffectorTwistsToObject)))
  'MapEffectorTwistsToObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapEffectorTwistsToObject)))
  'MapEffectorTwistsToObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapEffectorTwistsToObject)))
  "Returns string type for a service object of type '<MapEffectorTwistsToObject>"
  "manipulation_transforms/MapEffectorTwistsToObject")