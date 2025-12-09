; Auto-generated. Do not edit!


(cl:in-package nlp_drone_control-msg)


;//! \htmlinclude ToolCall.msg.html

(cl:defclass <ToolCall> (roslisp-msg-protocol:ros-message)
  ((function_name
    :reader function_name
    :initarg :function_name
    :type cl:string
    :initform "")
   (arguments_json
    :reader arguments_json
    :initarg :arguments_json
    :type cl:string
    :initform ""))
)

(cl:defclass ToolCall (<ToolCall>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToolCall>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToolCall)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nlp_drone_control-msg:<ToolCall> is deprecated: use nlp_drone_control-msg:ToolCall instead.")))

(cl:ensure-generic-function 'function_name-val :lambda-list '(m))
(cl:defmethod function_name-val ((m <ToolCall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-msg:function_name-val is deprecated.  Use nlp_drone_control-msg:function_name instead.")
  (function_name m))

(cl:ensure-generic-function 'arguments_json-val :lambda-list '(m))
(cl:defmethod arguments_json-val ((m <ToolCall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-msg:arguments_json-val is deprecated.  Use nlp_drone_control-msg:arguments_json instead.")
  (arguments_json m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToolCall>) ostream)
  "Serializes a message object of type '<ToolCall>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'function_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'function_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'arguments_json))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'arguments_json))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToolCall>) istream)
  "Deserializes a message object of type '<ToolCall>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'function_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'function_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arguments_json) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'arguments_json) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToolCall>)))
  "Returns string type for a message object of type '<ToolCall>"
  "nlp_drone_control/ToolCall")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToolCall)))
  "Returns string type for a message object of type 'ToolCall"
  "nlp_drone_control/ToolCall")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToolCall>)))
  "Returns md5sum for a message object of type '<ToolCall>"
  "45955b02be53f69e3133212fb3d15576")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToolCall)))
  "Returns md5sum for a message object of type 'ToolCall"
  "45955b02be53f69e3133212fb3d15576")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToolCall>)))
  "Returns full string definition for message of type '<ToolCall>"
  (cl:format cl:nil "# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToolCall)))
  "Returns full string definition for message of type 'ToolCall"
  (cl:format cl:nil "# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToolCall>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'function_name))
     4 (cl:length (cl:slot-value msg 'arguments_json))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToolCall>))
  "Converts a ROS message object to a list"
  (cl:list 'ToolCall
    (cl:cons ':function_name (function_name msg))
    (cl:cons ':arguments_json (arguments_json msg))
))
