; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapObjectPoseToEffectors-request.msg.html

(cl:defclass <MapObjectPoseToEffectors-request> (roslisp-msg-protocol:ros-message)
  ((object_pose
    :reader object_pose
    :initarg :object_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass MapObjectPoseToEffectors-request (<MapObjectPoseToEffectors-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectPoseToEffectors-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectPoseToEffectors-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectPoseToEffectors-request> is deprecated: use manipulation_transforms-srv:MapObjectPoseToEffectors-request instead.")))

(cl:ensure-generic-function 'object_pose-val :lambda-list '(m))
(cl:defmethod object_pose-val ((m <MapObjectPoseToEffectors-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_pose-val is deprecated.  Use manipulation_transforms-srv:object_pose instead.")
  (object_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectPoseToEffectors-request>) ostream)
  "Serializes a message object of type '<MapObjectPoseToEffectors-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectPoseToEffectors-request>) istream)
  "Deserializes a message object of type '<MapObjectPoseToEffectors-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectPoseToEffectors-request>)))
  "Returns string type for a service object of type '<MapObjectPoseToEffectors-request>"
  "manipulation_transforms/MapObjectPoseToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectPoseToEffectors-request)))
  "Returns string type for a service object of type 'MapObjectPoseToEffectors-request"
  "manipulation_transforms/MapObjectPoseToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectPoseToEffectors-request>)))
  "Returns md5sum for a message object of type '<MapObjectPoseToEffectors-request>"
  "fe420bd1f193579a27ccff2bf656fb54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectPoseToEffectors-request)))
  "Returns md5sum for a message object of type 'MapObjectPoseToEffectors-request"
  "fe420bd1f193579a27ccff2bf656fb54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectPoseToEffectors-request>)))
  "Returns full string definition for message of type '<MapObjectPoseToEffectors-request>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped object_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectPoseToEffectors-request)))
  "Returns full string definition for message of type 'MapObjectPoseToEffectors-request"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped object_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectPoseToEffectors-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectPoseToEffectors-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectPoseToEffectors-request
    (cl:cons ':object_pose (object_pose msg))
))
;//! \htmlinclude MapObjectPoseToEffectors-response.msg.html

(cl:defclass <MapObjectPoseToEffectors-response> (roslisp-msg-protocol:ros-message)
  ((effector_poses
    :reader effector_poses
    :initarg :effector_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped))))
)

(cl:defclass MapObjectPoseToEffectors-response (<MapObjectPoseToEffectors-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectPoseToEffectors-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectPoseToEffectors-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectPoseToEffectors-response> is deprecated: use manipulation_transforms-srv:MapObjectPoseToEffectors-response instead.")))

(cl:ensure-generic-function 'effector_poses-val :lambda-list '(m))
(cl:defmethod effector_poses-val ((m <MapObjectPoseToEffectors-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_poses-val is deprecated.  Use manipulation_transforms-srv:effector_poses instead.")
  (effector_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectPoseToEffectors-response>) ostream)
  "Serializes a message object of type '<MapObjectPoseToEffectors-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'effector_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'effector_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectPoseToEffectors-response>) istream)
  "Deserializes a message object of type '<MapObjectPoseToEffectors-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectPoseToEffectors-response>)))
  "Returns string type for a service object of type '<MapObjectPoseToEffectors-response>"
  "manipulation_transforms/MapObjectPoseToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectPoseToEffectors-response)))
  "Returns string type for a service object of type 'MapObjectPoseToEffectors-response"
  "manipulation_transforms/MapObjectPoseToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectPoseToEffectors-response>)))
  "Returns md5sum for a message object of type '<MapObjectPoseToEffectors-response>"
  "fe420bd1f193579a27ccff2bf656fb54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectPoseToEffectors-response)))
  "Returns md5sum for a message object of type 'MapObjectPoseToEffectors-response"
  "fe420bd1f193579a27ccff2bf656fb54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectPoseToEffectors-response>)))
  "Returns full string definition for message of type '<MapObjectPoseToEffectors-response>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] effector_poses~%~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectPoseToEffectors-response)))
  "Returns full string definition for message of type 'MapObjectPoseToEffectors-response"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] effector_poses~%~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectPoseToEffectors-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'effector_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectPoseToEffectors-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectPoseToEffectors-response
    (cl:cons ':effector_poses (effector_poses msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapObjectPoseToEffectors)))
  'MapObjectPoseToEffectors-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapObjectPoseToEffectors)))
  'MapObjectPoseToEffectors-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectPoseToEffectors)))
  "Returns string type for a service object of type '<MapObjectPoseToEffectors>"
  "manipulation_transforms/MapObjectPoseToEffectors")