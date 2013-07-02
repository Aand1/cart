; Auto-generated. Do not edit!


(cl:in-package cart_local_planner-msg)


;//! \htmlinclude pose2D.msg.html

(cl:defclass <pose2D> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0))
)

(cl:defclass pose2D (<pose2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cart_local_planner-msg:<pose2D> is deprecated: use cart_local_planner-msg:pose2D instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <pose2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cart_local_planner-msg:x-val is deprecated.  Use cart_local_planner-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <pose2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cart_local_planner-msg:y-val is deprecated.  Use cart_local_planner-msg:y instead.")
  (y m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <pose2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cart_local_planner-msg:t-val is deprecated.  Use cart_local_planner-msg:t instead.")
  (t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose2D>) ostream)
  "Serializes a message object of type '<pose2D>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose2D>) istream)
  "Deserializes a message object of type '<pose2D>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose2D>)))
  "Returns string type for a message object of type '<pose2D>"
  "cart_local_planner/pose2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose2D)))
  "Returns string type for a message object of type 'pose2D"
  "cart_local_planner/pose2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose2D>)))
  "Returns md5sum for a message object of type '<pose2D>"
  "35ad9130386373e3ccc4c834c109c59c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose2D)))
  "Returns md5sum for a message object of type 'pose2D"
  "35ad9130386373e3ccc4c834c109c59c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose2D>)))
  "Returns full string definition for message of type '<pose2D>"
  (cl:format cl:nil "# x,y,theta~%float32 x~%float32 y~%float32 t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose2D)))
  "Returns full string definition for message of type 'pose2D"
  (cl:format cl:nil "# x,y,theta~%float32 x~%float32 y~%float32 t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose2D>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose2D>))
  "Converts a ROS message object to a list"
  (cl:list 'pose2D
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':t (t msg))
))
