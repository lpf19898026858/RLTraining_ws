; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude DoneInfo.msg.html

(cl:defclass <DoneInfo> (roslisp-msg-protocol:ros-message)
  ((is_done
    :reader is_done
    :initarg :is_done
    :type cl:boolean
    :initform cl:nil)
   (is_success
    :reader is_success
    :initarg :is_success
    :type cl:boolean
    :initform cl:nil)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass DoneInfo (<DoneInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DoneInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DoneInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<DoneInfo> is deprecated: use custom_msgs-msg:DoneInfo instead.")))

(cl:ensure-generic-function 'is_done-val :lambda-list '(m))
(cl:defmethod is_done-val ((m <DoneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:is_done-val is deprecated.  Use custom_msgs-msg:is_done instead.")
  (is_done m))

(cl:ensure-generic-function 'is_success-val :lambda-list '(m))
(cl:defmethod is_success-val ((m <DoneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:is_success-val is deprecated.  Use custom_msgs-msg:is_success instead.")
  (is_success m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <DoneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:reason-val is deprecated.  Use custom_msgs-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DoneInfo>) ostream)
  "Serializes a message object of type '<DoneInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_done) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DoneInfo>) istream)
  "Deserializes a message object of type '<DoneInfo>"
    (cl:setf (cl:slot-value msg 'is_done) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DoneInfo>)))
  "Returns string type for a message object of type '<DoneInfo>"
  "custom_msgs/DoneInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DoneInfo)))
  "Returns string type for a message object of type 'DoneInfo"
  "custom_msgs/DoneInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DoneInfo>)))
  "Returns md5sum for a message object of type '<DoneInfo>"
  "f554d70bf7374158f003bce0d316e6fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DoneInfo)))
  "Returns md5sum for a message object of type 'DoneInfo"
  "f554d70bf7374158f003bce0d316e6fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DoneInfo>)))
  "Returns full string definition for message of type '<DoneInfo>"
  (cl:format cl:nil "bool is_done~%bool is_success~%string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DoneInfo)))
  "Returns full string definition for message of type 'DoneInfo"
  (cl:format cl:nil "bool is_done~%bool is_success~%string reason~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DoneInfo>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DoneInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DoneInfo
    (cl:cons ':is_done (is_done msg))
    (cl:cons ':is_success (is_success msg))
    (cl:cons ':reason (reason msg))
))
