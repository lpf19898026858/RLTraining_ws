; Auto-generated. Do not edit!


(cl:in-package nlp_drone_control-srv)


;//! \htmlinclude ExecuteDroneAction-request.msg.html

(cl:defclass <ExecuteDroneAction-request> (roslisp-msg-protocol:ros-message)
  ((action_type
    :reader action_type
    :initarg :action_type
    :type cl:string
    :initform "")
   (params_json
    :reader params_json
    :initarg :params_json
    :type cl:string
    :initform ""))
)

(cl:defclass ExecuteDroneAction-request (<ExecuteDroneAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteDroneAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteDroneAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nlp_drone_control-srv:<ExecuteDroneAction-request> is deprecated: use nlp_drone_control-srv:ExecuteDroneAction-request instead.")))

(cl:ensure-generic-function 'action_type-val :lambda-list '(m))
(cl:defmethod action_type-val ((m <ExecuteDroneAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-srv:action_type-val is deprecated.  Use nlp_drone_control-srv:action_type instead.")
  (action_type m))

(cl:ensure-generic-function 'params_json-val :lambda-list '(m))
(cl:defmethod params_json-val ((m <ExecuteDroneAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-srv:params_json-val is deprecated.  Use nlp_drone_control-srv:params_json instead.")
  (params_json m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteDroneAction-request>) ostream)
  "Serializes a message object of type '<ExecuteDroneAction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'params_json))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'params_json))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteDroneAction-request>) istream)
  "Deserializes a message object of type '<ExecuteDroneAction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'params_json) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'params_json) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteDroneAction-request>)))
  "Returns string type for a service object of type '<ExecuteDroneAction-request>"
  "nlp_drone_control/ExecuteDroneActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDroneAction-request)))
  "Returns string type for a service object of type 'ExecuteDroneAction-request"
  "nlp_drone_control/ExecuteDroneActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteDroneAction-request>)))
  "Returns md5sum for a message object of type '<ExecuteDroneAction-request>"
  "77584cf99b7cb35fa13143d3f49ac104")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteDroneAction-request)))
  "Returns md5sum for a message object of type 'ExecuteDroneAction-request"
  "77584cf99b7cb35fa13143d3f49ac104")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteDroneAction-request>)))
  "Returns full string definition for message of type '<ExecuteDroneAction-request>"
  (cl:format cl:nil "# --- Request ---~%# 动作的类型，例如 \"adjust_camera\", \"rotate_360\", \"hover_and_monitor\"~%string action_type~%~%# (可选) 动作的参数，我们可以用一个JSON字符串来灵活地传递~%# 例如：对于adjust_camera, params可以是 \"{\\\"pitch_angle\\\": -90.0}\"~%# 对于hover_and_monitor, params可以是 \"{\\\"duration_seconds\\\": 5.0}\"~%string params_json~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteDroneAction-request)))
  "Returns full string definition for message of type 'ExecuteDroneAction-request"
  (cl:format cl:nil "# --- Request ---~%# 动作的类型，例如 \"adjust_camera\", \"rotate_360\", \"hover_and_monitor\"~%string action_type~%~%# (可选) 动作的参数，我们可以用一个JSON字符串来灵活地传递~%# 例如：对于adjust_camera, params可以是 \"{\\\"pitch_angle\\\": -90.0}\"~%# 对于hover_and_monitor, params可以是 \"{\\\"duration_seconds\\\": 5.0}\"~%string params_json~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteDroneAction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action_type))
     4 (cl:length (cl:slot-value msg 'params_json))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteDroneAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteDroneAction-request
    (cl:cons ':action_type (action_type msg))
    (cl:cons ':params_json (params_json msg))
))
;//! \htmlinclude ExecuteDroneAction-response.msg.html

(cl:defclass <ExecuteDroneAction-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ExecuteDroneAction-response (<ExecuteDroneAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteDroneAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteDroneAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nlp_drone_control-srv:<ExecuteDroneAction-response> is deprecated: use nlp_drone_control-srv:ExecuteDroneAction-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecuteDroneAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-srv:success-val is deprecated.  Use nlp_drone_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ExecuteDroneAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nlp_drone_control-srv:message-val is deprecated.  Use nlp_drone_control-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteDroneAction-response>) ostream)
  "Serializes a message object of type '<ExecuteDroneAction-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteDroneAction-response>) istream)
  "Deserializes a message object of type '<ExecuteDroneAction-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteDroneAction-response>)))
  "Returns string type for a service object of type '<ExecuteDroneAction-response>"
  "nlp_drone_control/ExecuteDroneActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDroneAction-response)))
  "Returns string type for a service object of type 'ExecuteDroneAction-response"
  "nlp_drone_control/ExecuteDroneActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteDroneAction-response>)))
  "Returns md5sum for a message object of type '<ExecuteDroneAction-response>"
  "77584cf99b7cb35fa13143d3f49ac104")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteDroneAction-response)))
  "Returns md5sum for a message object of type 'ExecuteDroneAction-response"
  "77584cf99b7cb35fa13143d3f49ac104")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteDroneAction-response>)))
  "Returns full string definition for message of type '<ExecuteDroneAction-response>"
  (cl:format cl:nil "# --- Response ---~%bool success          # 动作是否成功执行~%string message        # 返回给LLM的信息，例如 \"Camera adjusted.\" 或 \"Rotation failed.\"~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteDroneAction-response)))
  "Returns full string definition for message of type 'ExecuteDroneAction-response"
  (cl:format cl:nil "# --- Response ---~%bool success          # 动作是否成功执行~%string message        # 返回给LLM的信息，例如 \"Camera adjusted.\" 或 \"Rotation failed.\"~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteDroneAction-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteDroneAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteDroneAction-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteDroneAction)))
  'ExecuteDroneAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteDroneAction)))
  'ExecuteDroneAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteDroneAction)))
  "Returns string type for a service object of type '<ExecuteDroneAction>"
  "nlp_drone_control/ExecuteDroneAction")