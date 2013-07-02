; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude MapObjectTrajectoryToEffectors-request.msg.html

(cl:defclass <MapObjectTrajectoryToEffectors-request> (roslisp-msg-protocol:ros-message)
  ((object_poses
    :reader object_poses
    :initarg :object_poses
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (object_twists
    :reader object_twists
    :initarg :object_twists
    :type (cl:vector geometry_msgs-msg:TwistStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TwistStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TwistStamped))))
)

(cl:defclass MapObjectTrajectoryToEffectors-request (<MapObjectTrajectoryToEffectors-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectTrajectoryToEffectors-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectTrajectoryToEffectors-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectTrajectoryToEffectors-request> is deprecated: use manipulation_transforms-srv:MapObjectTrajectoryToEffectors-request instead.")))

(cl:ensure-generic-function 'object_poses-val :lambda-list '(m))
(cl:defmethod object_poses-val ((m <MapObjectTrajectoryToEffectors-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_poses-val is deprecated.  Use manipulation_transforms-srv:object_poses instead.")
  (object_poses m))

(cl:ensure-generic-function 'object_twists-val :lambda-list '(m))
(cl:defmethod object_twists-val ((m <MapObjectTrajectoryToEffectors-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:object_twists-val is deprecated.  Use manipulation_transforms-srv:object_twists instead.")
  (object_twists m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectTrajectoryToEffectors-request>) ostream)
  "Serializes a message object of type '<MapObjectTrajectoryToEffectors-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectTrajectoryToEffectors-request>) istream)
  "Deserializes a message object of type '<MapObjectTrajectoryToEffectors-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectTrajectoryToEffectors-request>)))
  "Returns string type for a service object of type '<MapObjectTrajectoryToEffectors-request>"
  "manipulation_transforms/MapObjectTrajectoryToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTrajectoryToEffectors-request)))
  "Returns string type for a service object of type 'MapObjectTrajectoryToEffectors-request"
  "manipulation_transforms/MapObjectTrajectoryToEffectorsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectTrajectoryToEffectors-request>)))
  "Returns md5sum for a message object of type '<MapObjectTrajectoryToEffectors-request>"
  "4826779f151f916da731426fc4355b7a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectTrajectoryToEffectors-request)))
  "Returns md5sum for a message object of type 'MapObjectTrajectoryToEffectors-request"
  "4826779f151f916da731426fc4355b7a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectTrajectoryToEffectors-request>)))
  "Returns full string definition for message of type '<MapObjectTrajectoryToEffectors-request>"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] object_poses~%geometry_msgs/TwistStamped[] object_twists~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectTrajectoryToEffectors-request)))
  "Returns full string definition for message of type 'MapObjectTrajectoryToEffectors-request"
  (cl:format cl:nil "~%geometry_msgs/PoseStamped[] object_poses~%geometry_msgs/TwistStamped[] object_twists~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectTrajectoryToEffectors-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object_twists) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectTrajectoryToEffectors-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectTrajectoryToEffectors-request
    (cl:cons ':object_poses (object_poses msg))
    (cl:cons ':object_twists (object_twists msg))
))
;//! \htmlinclude MapObjectTrajectoryToEffectors-response.msg.html

(cl:defclass <MapObjectTrajectoryToEffectors-response> (roslisp-msg-protocol:ros-message)
  ((effector_traj
    :reader effector_traj
    :initarg :effector_traj
    :type manipulation_transforms-msg:EffectorTrajectories
    :initform (cl:make-instance 'manipulation_transforms-msg:EffectorTrajectories)))
)

(cl:defclass MapObjectTrajectoryToEffectors-response (<MapObjectTrajectoryToEffectors-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapObjectTrajectoryToEffectors-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapObjectTrajectoryToEffectors-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<MapObjectTrajectoryToEffectors-response> is deprecated: use manipulation_transforms-srv:MapObjectTrajectoryToEffectors-response instead.")))

(cl:ensure-generic-function 'effector_traj-val :lambda-list '(m))
(cl:defmethod effector_traj-val ((m <MapObjectTrajectoryToEffectors-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:effector_traj-val is deprecated.  Use manipulation_transforms-srv:effector_traj instead.")
  (effector_traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapObjectTrajectoryToEffectors-response>) ostream)
  "Serializes a message object of type '<MapObjectTrajectoryToEffectors-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'effector_traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapObjectTrajectoryToEffectors-response>) istream)
  "Deserializes a message object of type '<MapObjectTrajectoryToEffectors-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'effector_traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapObjectTrajectoryToEffectors-response>)))
  "Returns string type for a service object of type '<MapObjectTrajectoryToEffectors-response>"
  "manipulation_transforms/MapObjectTrajectoryToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTrajectoryToEffectors-response)))
  "Returns string type for a service object of type 'MapObjectTrajectoryToEffectors-response"
  "manipulation_transforms/MapObjectTrajectoryToEffectorsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapObjectTrajectoryToEffectors-response>)))
  "Returns md5sum for a message object of type '<MapObjectTrajectoryToEffectors-response>"
  "4826779f151f916da731426fc4355b7a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapObjectTrajectoryToEffectors-response)))
  "Returns md5sum for a message object of type 'MapObjectTrajectoryToEffectors-response"
  "4826779f151f916da731426fc4355b7a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapObjectTrajectoryToEffectors-response>)))
  "Returns full string definition for message of type '<MapObjectTrajectoryToEffectors-response>"
  (cl:format cl:nil "~%EffectorTrajectories effector_traj~%~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectories~%# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapObjectTrajectoryToEffectors-response)))
  "Returns full string definition for message of type 'MapObjectTrajectoryToEffectors-response"
  (cl:format cl:nil "~%EffectorTrajectories effector_traj~%~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectories~%# Array of EffectorTrajectoriesPoints of arbitrary length~%EffectorTrajectoriesPoint[] pt~%~%# Optional effector names~%string[] names~%~%================================================================================~%MSG: manipulation_transforms/EffectorTrajectoriesPoint~%# Array of poses, one for each effector~%geometry_msgs/PoseStamped[] poses~%~%# Array of twists, one for each effector~%geometry_msgs/TwistStamped[] twists~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapObjectTrajectoryToEffectors-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'effector_traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapObjectTrajectoryToEffectors-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MapObjectTrajectoryToEffectors-response
    (cl:cons ':effector_traj (effector_traj msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MapObjectTrajectoryToEffectors)))
  'MapObjectTrajectoryToEffectors-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MapObjectTrajectoryToEffectors)))
  'MapObjectTrajectoryToEffectors-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapObjectTrajectoryToEffectors)))
  "Returns string type for a service object of type '<MapObjectTrajectoryToEffectors>"
  "manipulation_transforms/MapObjectTrajectoryToEffectors")