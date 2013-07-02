; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapObjectTwistToEffectors-request.msg.html

(cl:defclass <MapObjectTwistToEffectors-request> (roslisp-msg-protocol:ros-message)
  ((object_twist
    :reader object_twist
    :initarg :object_twist
    :type geometry_msgs-msg:TwistStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TwistStamped)))
)

(cl:defclass MapObjectTwistToEffectors-request (<MapObjectTwistToEffectors-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectTwistToEffectors-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectTwistToEffectors-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectTwistToEffectors-request> is deprecated: use manipulation_transforms-srv:MapObjectTwistToEffectors-request instead.")))

(cl:ensure-generic-function 'object_twist-val :lambda-list '(m))
(cl:defmethod object_twist-val ((m <MapObjectTwistToEffectors-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_twist-val is deprecated.  Use manipulation_transforms-srv:object_twist instead.")
  (object_twist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectTwistToEffectors-request>) ostream)
  "Serializes a message object of type '<MapObjectTwistToEffectors-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_twist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectTwistToEffectors-request>) istream)
  "Deserializes a message object of type '<MapObjectTwistToEffectors-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_twist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectTwistToEffectors-request>)))
  "Returns string type for a service object of type '<MapObjectTwistToEffectors-request>"
  "manipulation_transforms/MapObjectTwistToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTwistToEffectors-request)))
  "Returns string type for a service object of type 'MapObjectTwistToEffectors-request"
  "manipulation_transforms/MapObjectTwistToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectTwistToEffectors-request>)))
  "Returns md5sum for a message object of type '<MapObjectTwistToEffectors-request>"
  "1bb5479e5565270d07bd4dd38bdd26c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectTwistToEffectors-request)))
  "Returns md5sum for a message object of type 'MapObjectTwistToEffectors-request"
  "1bb5479e5565270d07bd4dd38bdd26c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectTwistToEffectors-request>)))
  "Returns full string definition for message of type '<MapObjectTwistToEffectors-request>"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped object_twist~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectTwistToEffectors-request)))
  "Returns full string definition for message of type 'MapObjectTwistToEffectors-request"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped object_twist~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectTwistToEffectors-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_twist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectTwistToEffectors-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectTwistToEffectors-request
    (cl:cons ':object_twist (object_twist msg))
))
;//! \htmlinclude MapObjectTwistToEffectors-response.msg.html

(cl:defclass <MapObjectTwistToEffectors-response> (roslisp-msg-protocol:ros-message)
  ((effector_twists
    :reader effector_twists
    :initarg :effector_twists
    :type (cl:vector geometry_msgs-msg:TwistStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TwistStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TwistStamped))))
)

(cl:defclass MapObjectTwistToEffectors-response (<MapObjectTwistToEffectors-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectTwistToEffectors-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectTwistToEffectors-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectTwistToEffectors-response> is deprecated: use manipulation_transforms-srv:MapObjectTwistToEffectors-response instead.")))

(cl:ensure-generic-function 'effector_twists-val :lambda-list '(m))
(cl:defmethod effector_twists-val ((m <MapObjectTwistToEffectors-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_twists-val is deprecated.  Use manipulation_transforms-srv:effector_twists instead.")
  (effector_twists m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectTwistToEffectors-response>) ostream)
  "Serializes a message object of type '<MapObjectTwistToEffectors-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'effector_twists))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'effector_twists))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectTwistToEffectors-response>) istream)
  "Deserializes a message object of type '<MapObjectTwistToEffectors-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectTwistToEffectors-response>)))
  "Returns string type for a service object of type '<MapObjectTwistToEffectors-response>"
  "manipulation_transforms/MapObjectTwistToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTwistToEffectors-response)))
  "Returns string type for a service object of type 'MapObjectTwistToEffectors-response"
  "manipulation_transforms/MapObjectTwistToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectTwistToEffectors-response>)))
  "Returns md5sum for a message object of type '<MapObjectTwistToEffectors-response>"
  "1bb5479e5565270d07bd4dd38bdd26c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectTwistToEffectors-response)))
  "Returns md5sum for a message object of type 'MapObjectTwistToEffectors-response"
  "1bb5479e5565270d07bd4dd38bdd26c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectTwistToEffectors-response>)))
  "Returns full string definition for message of type '<MapObjectTwistToEffectors-response>"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped[] effector_twists~%~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectTwistToEffectors-response)))
  "Returns full string definition for message of type 'MapObjectTwistToEffectors-response"
  (cl:format cl:nil "~%geometry_msgs/TwistStamped[] effector_twists~%~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectTwistToEffectors-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'effector_twists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectTwistToEffectors-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectTwistToEffectors-response
    (cl:cons ':effector_twists (effector_twists msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapObjectTwistToEffectors)))
  'MapObjectTwistToEffectors-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapObjectTwistToEffectors)))
  'MapObjectTwistToEffectors-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTwistToEffectors)))
  "Returns string type for a service object of type '<MapObjectTwistToEffectors>"
  "manipulation_transforms/MapObjectTwistToEffectors")