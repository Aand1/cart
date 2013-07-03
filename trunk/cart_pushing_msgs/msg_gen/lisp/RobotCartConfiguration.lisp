; Auto-generated. Do not edit!


(cl:in-package cart_pushing_msgs-msg)


;//! \htmlinclude RobotCartConfiguration.msg.html

(cl:defclass <RobotCartConfiguration> (roslisp-msg-protocol:ros-message)
  ((robot_pose
    :reader robot_pose
    :initarg :robot_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (cart_pose
    :reader cart_pose
    :initarg :cart_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass RobotCartConfiguration (<RobotCartConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCartConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCartConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cart_pushing_msgs-msg:<RobotCartConfiguration> is deprecated: use cart_pushing_msgs-msg:RobotCartConfiguration instead.")))

(cl:ensure-generic-function 'robot_pose-val :lambda-list '(m))
(cl:defmethod robot_pose-val ((m <RobotCartConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cart_pushing_msgs-msg:robot_pose-val is deprecated.  Use cart_pushing_msgs-msg:robot_pose instead.")
  (robot_pose m))

(cl:ensure-generic-function 'cart_pose-val :lambda-list '(m))
(cl:defmethod cart_pose-val ((m <RobotCartConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cart_pushing_msgs-msg:cart_pose-val is deprecated.  Use cart_pushing_msgs-msg:cart_pose instead.")
  (cart_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCartConfiguration>) ostream)
  "Serializes a message object of type '<RobotCartConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cart_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCartConfiguration>) istream)
  "Deserializes a message object of type '<RobotCartConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cart_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCartConfiguration>)))
  "Returns string type for a message object of type '<RobotCartConfiguration>"
  "cart_pushing_msgs/RobotCartConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCartConfiguration)))
  "Returns string type for a message object of type 'RobotCartConfiguration"
  "cart_pushing_msgs/RobotCartConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCartConfiguration>)))
  "Returns md5sum for a message object of type '<RobotCartConfiguration>"
  "b6763adc7a4163e81b79c0de27053f06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCartConfiguration)))
  "Returns md5sum for a message object of type 'RobotCartConfiguration"
  "b6763adc7a4163e81b79c0de27053f06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCartConfiguration>)))
  "Returns full string definition for message of type '<RobotCartConfiguration>"
  (cl:format cl:nil "# Robot's pose in reference frame~%geometry_msgs/Pose robot_pose~%~%# Cart's pose in base frame~%geometry_msgs/Pose cart_pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCartConfiguration)))
  "Returns full string definition for message of type 'RobotCartConfiguration"
  (cl:format cl:nil "# Robot's pose in reference frame~%geometry_msgs/Pose robot_pose~%~%# Cart's pose in base frame~%geometry_msgs/Pose cart_pose~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCartConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cart_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCartConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCartConfiguration
    (cl:cons ':robot_pose (robot_pose msg))
    (cl:cons ':cart_pose (cart_pose msg))
))
