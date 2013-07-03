; Auto-generated. Do not edit!


(cl:in-package sbpl_cart_planner-msg)


;//! \htmlinclude SBPLCartPlannerStats.msg.html

(cl:defclass <SBPLCartPlannerStats> (roslisp-msg-protocol:ros-message)
  ((initial_epsilon
    :reader initial_epsilon
    :initarg :initial_epsilon
    :type cl:float
    :initform 0.0)
   (final_epsilon
    :reader final_epsilon
    :initarg :final_epsilon
    :type cl:float
    :initform 0.0)
   (plan_to_first_solution
    :reader plan_to_first_solution
    :initarg :plan_to_first_solution
    :type cl:boolean
    :initform cl:nil)
   (allocated_time
    :reader allocated_time
    :initarg :allocated_time
    :type cl:float
    :initform 0.0)
   (actual_time
    :reader actual_time
    :initarg :actual_time
    :type cl:float
    :initform 0.0)
   (time_to_first_solution
    :reader time_to_first_solution
    :initarg :time_to_first_solution
    :type cl:float
    :initform 0.0)
   (solution_cost
    :reader solution_cost
    :initarg :solution_cost
    :type cl:float
    :initform 0.0)
   (path_size
    :reader path_size
    :initarg :path_size
    :type cl:float
    :initform 0.0)
   (final_number_of_expands
    :reader final_number_of_expands
    :initarg :final_number_of_expands
    :type cl:integer
    :initform 0)
   (number_of_expands_initial_solution
    :reader number_of_expands_initial_solution
    :initarg :number_of_expands_initial_solution
    :type cl:integer
    :initform 0)
   (start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (start_cart_angle
    :reader start_cart_angle
    :initarg :start_cart_angle
    :type cl:float
    :initform 0.0)
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (goal_cart_angle
    :reader goal_cart_angle
    :initarg :goal_cart_angle
    :type cl:float
    :initform 0.0)
   (solution
    :reader solution
    :initarg :solution
    :type cart_pushing_msgs-msg:RobotCartPath
    :initform (cl:make-instance 'cart_pushing_msgs-msg:RobotCartPath)))
)

(cl:defclass SBPLCartPlannerStats (<SBPLCartPlannerStats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SBPLCartPlannerStats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SBPLCartPlannerStats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sbpl_cart_planner-msg:<SBPLCartPlannerStats> is deprecated: use sbpl_cart_planner-msg:SBPLCartPlannerStats instead.")))

(cl:ensure-generic-function 'initial_epsilon-val :lambda-list '(m))
(cl:defmethod initial_epsilon-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:initial_epsilon-val is deprecated.  Use sbpl_cart_planner-msg:initial_epsilon instead.")
  (initial_epsilon m))

(cl:ensure-generic-function 'final_epsilon-val :lambda-list '(m))
(cl:defmethod final_epsilon-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:final_epsilon-val is deprecated.  Use sbpl_cart_planner-msg:final_epsilon instead.")
  (final_epsilon m))

(cl:ensure-generic-function 'plan_to_first_solution-val :lambda-list '(m))
(cl:defmethod plan_to_first_solution-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:plan_to_first_solution-val is deprecated.  Use sbpl_cart_planner-msg:plan_to_first_solution instead.")
  (plan_to_first_solution m))

(cl:ensure-generic-function 'allocated_time-val :lambda-list '(m))
(cl:defmethod allocated_time-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:allocated_time-val is deprecated.  Use sbpl_cart_planner-msg:allocated_time instead.")
  (allocated_time m))

(cl:ensure-generic-function 'actual_time-val :lambda-list '(m))
(cl:defmethod actual_time-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:actual_time-val is deprecated.  Use sbpl_cart_planner-msg:actual_time instead.")
  (actual_time m))

(cl:ensure-generic-function 'time_to_first_solution-val :lambda-list '(m))
(cl:defmethod time_to_first_solution-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:time_to_first_solution-val is deprecated.  Use sbpl_cart_planner-msg:time_to_first_solution instead.")
  (time_to_first_solution m))

(cl:ensure-generic-function 'solution_cost-val :lambda-list '(m))
(cl:defmethod solution_cost-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:solution_cost-val is deprecated.  Use sbpl_cart_planner-msg:solution_cost instead.")
  (solution_cost m))

(cl:ensure-generic-function 'path_size-val :lambda-list '(m))
(cl:defmethod path_size-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:path_size-val is deprecated.  Use sbpl_cart_planner-msg:path_size instead.")
  (path_size m))

(cl:ensure-generic-function 'final_number_of_expands-val :lambda-list '(m))
(cl:defmethod final_number_of_expands-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:final_number_of_expands-val is deprecated.  Use sbpl_cart_planner-msg:final_number_of_expands instead.")
  (final_number_of_expands m))

(cl:ensure-generic-function 'number_of_expands_initial_solution-val :lambda-list '(m))
(cl:defmethod number_of_expands_initial_solution-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:number_of_expands_initial_solution-val is deprecated.  Use sbpl_cart_planner-msg:number_of_expands_initial_solution instead.")
  (number_of_expands_initial_solution m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:start-val is deprecated.  Use sbpl_cart_planner-msg:start instead.")
  (start m))

(cl:ensure-generic-function 'start_cart_angle-val :lambda-list '(m))
(cl:defmethod start_cart_angle-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:start_cart_angle-val is deprecated.  Use sbpl_cart_planner-msg:start_cart_angle instead.")
  (start_cart_angle m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:goal-val is deprecated.  Use sbpl_cart_planner-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'goal_cart_angle-val :lambda-list '(m))
(cl:defmethod goal_cart_angle-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:goal_cart_angle-val is deprecated.  Use sbpl_cart_planner-msg:goal_cart_angle instead.")
  (goal_cart_angle m))

(cl:ensure-generic-function 'solution-val :lambda-list '(m))
(cl:defmethod solution-val ((m <SBPLCartPlannerStats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sbpl_cart_planner-msg:solution-val is deprecated.  Use sbpl_cart_planner-msg:solution instead.")
  (solution m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SBPLCartPlannerStats>) ostream)
  "Serializes a message object of type '<SBPLCartPlannerStats>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'initial_epsilon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'final_epsilon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plan_to_first_solution) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'allocated_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'actual_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time_to_first_solution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'solution_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'path_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'final_number_of_expands)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'number_of_expands_initial_solution)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_cart_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'goal_cart_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'solution) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SBPLCartPlannerStats>) istream)
  "Deserializes a message object of type '<SBPLCartPlannerStats>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initial_epsilon) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'final_epsilon) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'plan_to_first_solution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'allocated_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'actual_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_to_first_solution) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'solution_cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'path_size) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'final_number_of_expands) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'number_of_expands_initial_solution) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_cart_angle) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'goal_cart_angle) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'solution) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SBPLCartPlannerStats>)))
  "Returns string type for a message object of type '<SBPLCartPlannerStats>"
  "sbpl_cart_planner/SBPLCartPlannerStats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SBPLCartPlannerStats)))
  "Returns string type for a message object of type 'SBPLCartPlannerStats"
  "sbpl_cart_planner/SBPLCartPlannerStats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SBPLCartPlannerStats>)))
  "Returns md5sum for a message object of type '<SBPLCartPlannerStats>"
  "59dcf49825e7e59499a38ffdd0daa999")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SBPLCartPlannerStats)))
  "Returns md5sum for a message object of type 'SBPLCartPlannerStats"
  "59dcf49825e7e59499a38ffdd0daa999")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SBPLCartPlannerStats>)))
  "Returns full string definition for message of type '<SBPLCartPlannerStats>"
  (cl:format cl:nil "#planner stats~%float64 initial_epsilon~%float64 final_epsilon~%bool plan_to_first_solution~%float64 allocated_time~%float64 actual_time~%float64 time_to_first_solution~%float64 solution_cost~%float64 path_size~%int64 final_number_of_expands~%int64 number_of_expands_initial_solution~%~%#problem stats~%geometry_msgs/PoseStamped start~%float64 start_cart_angle~%geometry_msgs/PoseStamped goal~%float64 goal_cart_angle~%~%#solution~%cart_pushing_msgs/RobotCartPath solution~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: cart_pushing_msgs/RobotCartPath~%Header header~%RobotCartConfiguration[] path~%================================================================================~%MSG: cart_pushing_msgs/RobotCartConfiguration~%# Robot's pose in reference frame~%geometry_msgs/Pose robot_pose~%~%# Cart's pose in base frame~%geometry_msgs/Pose cart_pose~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SBPLCartPlannerStats)))
  "Returns full string definition for message of type 'SBPLCartPlannerStats"
  (cl:format cl:nil "#planner stats~%float64 initial_epsilon~%float64 final_epsilon~%bool plan_to_first_solution~%float64 allocated_time~%float64 actual_time~%float64 time_to_first_solution~%float64 solution_cost~%float64 path_size~%int64 final_number_of_expands~%int64 number_of_expands_initial_solution~%~%#problem stats~%geometry_msgs/PoseStamped start~%float64 start_cart_angle~%geometry_msgs/PoseStamped goal~%float64 goal_cart_angle~%~%#solution~%cart_pushing_msgs/RobotCartPath solution~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: cart_pushing_msgs/RobotCartPath~%Header header~%RobotCartConfiguration[] path~%================================================================================~%MSG: cart_pushing_msgs/RobotCartConfiguration~%# Robot's pose in reference frame~%geometry_msgs/Pose robot_pose~%~%# Cart's pose in base frame~%geometry_msgs/Pose cart_pose~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SBPLCartPlannerStats>))
  (cl:+ 0
     8
     8
     1
     8
     8
     8
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'solution))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SBPLCartPlannerStats>))
  "Converts a ROS message object to a list"
  (cl:list 'SBPLCartPlannerStats
    (cl:cons ':initial_epsilon (initial_epsilon msg))
    (cl:cons ':final_epsilon (final_epsilon msg))
    (cl:cons ':plan_to_first_solution (plan_to_first_solution msg))
    (cl:cons ':allocated_time (allocated_time msg))
    (cl:cons ':actual_time (actual_time msg))
    (cl:cons ':time_to_first_solution (time_to_first_solution msg))
    (cl:cons ':solution_cost (solution_cost msg))
    (cl:cons ':path_size (path_size msg))
    (cl:cons ':final_number_of_expands (final_number_of_expands msg))
    (cl:cons ':number_of_expands_initial_solution (number_of_expands_initial_solution msg))
    (cl:cons ':start (start msg))
    (cl:cons ':start_cart_angle (start_cart_angle msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':goal_cart_angle (goal_cart_angle msg))
    (cl:cons ':solution (solution msg))
))
