; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLEvent.msg.html

(cl:defclass <RLEvent> (roslisp-msg-protocol:ros-message)
  ((event_type
    :reader event_type
    :initarg :event_type
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RLEvent (<RLEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLEvent> is deprecated: use rl_bridge-msg:RLEvent instead.")))

(cl:ensure-generic-function 'event_type-val :lambda-list '(m))
(cl:defmethod event_type-val ((m <RLEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:event_type-val is deprecated.  Use rl_bridge-msg:event_type instead.")
  (event_type m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <RLEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:data-val is deprecated.  Use rl_bridge-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <RLEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:done-val is deprecated.  Use rl_bridge-msg:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLEvent>) ostream)
  "Serializes a message object of type '<RLEvent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'event_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'event_type))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLEvent>) istream)
  "Deserializes a message object of type '<RLEvent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'event_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'event_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLEvent>)))
  "Returns string type for a message object of type '<RLEvent>"
  "rl_bridge/RLEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLEvent)))
  "Returns string type for a message object of type 'RLEvent"
  "rl_bridge/RLEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLEvent>)))
  "Returns md5sum for a message object of type '<RLEvent>"
  "3d11c09bdebe9d87801e4c84e2840393")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLEvent)))
  "Returns md5sum for a message object of type 'RLEvent"
  "3d11c09bdebe9d87801e4c84e2840393")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLEvent>)))
  "Returns full string definition for message of type '<RLEvent>"
  (cl:format cl:nil "# RLEvent.msg~%~%string event_type         # 事件类型，例如 \"move\", \"reached_target\", \"collision\", \"out_of_bounds\"~%float32[] data            # 附带数据，例如 [distance, velocity, angle]~%bool done                 # 是否终止当前 episode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLEvent)))
  "Returns full string definition for message of type 'RLEvent"
  (cl:format cl:nil "# RLEvent.msg~%~%string event_type         # 事件类型，例如 \"move\", \"reached_target\", \"collision\", \"out_of_bounds\"~%float32[] data            # 附带数据，例如 [distance, velocity, angle]~%bool done                 # 是否终止当前 episode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLEvent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'event_type))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'RLEvent
    (cl:cons ':event_type (event_type msg))
    (cl:cons ':data (data msg))
    (cl:cons ':done (done msg))
))
