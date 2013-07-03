; Auto-generated. Do not edit!


(cl:in-package manipulation_transforms-srv)


;//! \htmlinclude LoadInitialTransforms-request.msg.html

(cl:defclass <LoadInitialTransforms-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass LoadInitialTransforms-request (<LoadInitialTransforms-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadInitialTransforms-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadInitialTransforms-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<LoadInitialTransforms-request> is deprecated: use manipulation_transforms-srv:LoadInitialTransforms-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <LoadInitialTransforms-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:name-val is deprecated.  Use manipulation_transforms-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadInitialTransforms-request>) ostream)
  "Serializes a message object of type '<LoadInitialTransforms-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadInitialTransforms-request>) istream)
  "Deserializes a message object of type '<LoadInitialTransforms-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadInitialTransforms-request>)))
  "Returns string type for a service object of type '<LoadInitialTransforms-request>"
  "manipulation_transforms/LoadInitialTransformsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadInitialTransforms-request)))
  "Returns string type for a service object of type 'LoadInitialTransforms-request"
  "manipulation_transforms/LoadInitialTransformsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadInitialTransforms-request>)))
  "Returns md5sum for a message object of type '<LoadInitialTransforms-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadInitialTransforms-request)))
  "Returns md5sum for a message object of type 'LoadInitialTransforms-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadInitialTransforms-request>)))
  "Returns full string definition for message of type '<LoadInitialTransforms-request>"
  (cl:format cl:nil "~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadInitialTransforms-request)))
  "Returns full string definition for message of type 'LoadInitialTransforms-request"
  (cl:format cl:nil "~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadInitialTransforms-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadInitialTransforms-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadInitialTransforms-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude LoadInitialTransforms-response.msg.html

(cl:defclass <LoadInitialTransforms-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadInitialTransforms-response (<LoadInitialTransforms-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadInitialTransforms-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadInitialTransforms-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manipulation_transforms-srv:<LoadInitialTransforms-response> is deprecated: use manipulation_transforms-srv:LoadInitialTransforms-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadInitialTransforms-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manipulation_transforms-srv:success-val is deprecated.  Use manipulation_transforms-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadInitialTransforms-response>) ostream)
  "Serializes a message object of type '<LoadInitialTransforms-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadInitialTransforms-response>) istream)
  "Deserializes a message object of type '<LoadInitialTransforms-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadInitialTransforms-response>)))
  "Returns string type for a service object of type '<LoadInitialTransforms-response>"
  "manipulation_transforms/LoadInitialTransformsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadInitialTransforms-response)))
  "Returns string type for a service object of type 'LoadInitialTransforms-response"
  "manipulation_transforms/LoadInitialTransformsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadInitialTransforms-response>)))
  "Returns md5sum for a message object of type '<LoadInitialTransforms-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadInitialTransforms-response)))
  "Returns md5sum for a message object of type 'LoadInitialTransforms-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadInitialTransforms-response>)))
  "Returns full string definition for message of type '<LoadInitialTransforms-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadInitialTransforms-response)))
  "Returns full string definition for message of type 'LoadInitialTransforms-response"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadInitialTransforms-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadInitialTransforms-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadInitialTransforms-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadInitialTransforms)))
  'LoadInitialTransforms-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadInitialTransforms)))
  'LoadInitialTransforms-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadInitialTransforms)))
  "Returns string type for a service object of type '<LoadInitialTransforms>"
  "manipulation_transforms/LoadInitialTransforms")