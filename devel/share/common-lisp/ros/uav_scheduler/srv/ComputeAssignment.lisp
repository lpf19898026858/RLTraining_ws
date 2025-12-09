; Auto-generated. Do not edit!


(cl:in-package uav_scheduler-srv)


;//! \htmlinclude ComputeAssignment-request.msg.html

(cl:defclass <ComputeAssignment-request> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:string
    :initform "")
   (drones
    :reader drones
    :initarg :drones
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (pois
    :reader pois
    :initarg :pois
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ComputeAssignment-request (<ComputeAssignment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeAssignment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeAssignment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_scheduler-srv:<ComputeAssignment-request> is deprecated: use uav_scheduler-srv:ComputeAssignment-request instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <ComputeAssignment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_scheduler-srv:algorithm-val is deprecated.  Use uav_scheduler-srv:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'drones-val :lambda-list '(m))
(cl:defmethod drones-val ((m <ComputeAssignment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_scheduler-srv:drones-val is deprecated.  Use uav_scheduler-srv:drones instead.")
  (drones m))

(cl:ensure-generic-function 'pois-val :lambda-list '(m))
(cl:defmethod pois-val ((m <ComputeAssignment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_scheduler-srv:pois-val is deprecated.  Use uav_scheduler-srv:pois instead.")
  (pois m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeAssignment-request>) ostream)
  "Serializes a message object of type '<ComputeAssignment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'algorithm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'algorithm))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'drones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'drones))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pois))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'pois))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeAssignment-request>) istream)
  "Deserializes a message object of type '<ComputeAssignment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'algorithm) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'algorithm) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'drones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'drones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pois) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pois)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeAssignment-request>)))
  "Returns string type for a service object of type '<ComputeAssignment-request>"
  "uav_scheduler/ComputeAssignmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeAssignment-request)))
  "Returns string type for a service object of type 'ComputeAssignment-request"
  "uav_scheduler/ComputeAssignmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeAssignment-request>)))
  "Returns md5sum for a message object of type '<ComputeAssignment-request>"
  "97370e26157faf720c13585eba94765b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeAssignment-request)))
  "Returns md5sum for a message object of type 'ComputeAssignment-request"
  "97370e26157faf720c13585eba94765b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeAssignment-request>)))
  "Returns full string definition for message of type '<ComputeAssignment-request>"
  (cl:format cl:nil "# 输入~%string algorithm              # \"hungarian\", \"auction\", \"genetic\", \"distributed_auction\"~%string[] drones~%string[] pois~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeAssignment-request)))
  "Returns full string definition for message of type 'ComputeAssignment-request"
  (cl:format cl:nil "# 输入~%string algorithm              # \"hungarian\", \"auction\", \"genetic\", \"distributed_auction\"~%string[] drones~%string[] pois~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeAssignment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'algorithm))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'drones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pois) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeAssignment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeAssignment-request
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':drones (drones msg))
    (cl:cons ':pois (pois msg))
))
;//! \htmlinclude ComputeAssignment-response.msg.html

(cl:defclass <ComputeAssignment-response> (roslisp-msg-protocol:ros-message)
  ((assignments
    :reader assignments
    :initarg :assignments
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (plan_text
    :reader plan_text
    :initarg :plan_text
    :type cl:string
    :initform ""))
)

(cl:defclass ComputeAssignment-response (<ComputeAssignment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeAssignment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeAssignment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uav_scheduler-srv:<ComputeAssignment-response> is deprecated: use uav_scheduler-srv:ComputeAssignment-response instead.")))

(cl:ensure-generic-function 'assignments-val :lambda-list '(m))
(cl:defmethod assignments-val ((m <ComputeAssignment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_scheduler-srv:assignments-val is deprecated.  Use uav_scheduler-srv:assignments instead.")
  (assignments m))

(cl:ensure-generic-function 'plan_text-val :lambda-list '(m))
(cl:defmethod plan_text-val ((m <ComputeAssignment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uav_scheduler-srv:plan_text-val is deprecated.  Use uav_scheduler-srv:plan_text instead.")
  (plan_text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeAssignment-response>) ostream)
  "Serializes a message object of type '<ComputeAssignment-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'assignments))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'assignments))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plan_text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plan_text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeAssignment-response>) istream)
  "Deserializes a message object of type '<ComputeAssignment-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'assignments) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'assignments)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan_text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'plan_text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeAssignment-response>)))
  "Returns string type for a service object of type '<ComputeAssignment-response>"
  "uav_scheduler/ComputeAssignmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeAssignment-response)))
  "Returns string type for a service object of type 'ComputeAssignment-response"
  "uav_scheduler/ComputeAssignmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeAssignment-response>)))
  "Returns md5sum for a message object of type '<ComputeAssignment-response>"
  "97370e26157faf720c13585eba94765b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeAssignment-response)))
  "Returns md5sum for a message object of type 'ComputeAssignment-response"
  "97370e26157faf720c13585eba94765b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeAssignment-response>)))
  "Returns full string definition for message of type '<ComputeAssignment-response>"
  (cl:format cl:nil "# 输出~%string[] assignments          # 匹配结果，例如 V_UAV_0->House1~%string plan_text              # 任务动作文本~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeAssignment-response)))
  "Returns full string definition for message of type 'ComputeAssignment-response"
  (cl:format cl:nil "# 输出~%string[] assignments          # 匹配结果，例如 V_UAV_0->House1~%string plan_text              # 任务动作文本~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeAssignment-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'assignments) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'plan_text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeAssignment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeAssignment-response
    (cl:cons ':assignments (assignments msg))
    (cl:cons ':plan_text (plan_text msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeAssignment)))
  'ComputeAssignment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeAssignment)))
  'ComputeAssignment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeAssignment)))
  "Returns string type for a service object of type '<ComputeAssignment>"
  "uav_scheduler/ComputeAssignment")