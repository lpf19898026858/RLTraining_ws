; Auto-generated. Do not edit!


(cl:in-package fleet_manager-srv)


;//! \htmlinclude InjectTasks-request.msg.html

(cl:defclass <InjectTasks-request> (roslisp-msg-protocol:ros-message)
  ((tasks
    :reader tasks
    :initarg :tasks
    :type (cl:vector fleet_manager-msg:TaskStatus)
   :initform (cl:make-array 0 :element-type 'fleet_manager-msg:TaskStatus :initial-element (cl:make-instance 'fleet_manager-msg:TaskStatus)))
   (assign_to_drone_id
    :reader assign_to_drone_id
    :initarg :assign_to_drone_id
    :type cl:string
    :initform ""))
)

(cl:defclass InjectTasks-request (<InjectTasks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InjectTasks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InjectTasks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_manager-srv:<InjectTasks-request> is deprecated: use fleet_manager-srv:InjectTasks-request instead.")))

(cl:ensure-generic-function 'tasks-val :lambda-list '(m))
(cl:defmethod tasks-val ((m <InjectTasks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-srv:tasks-val is deprecated.  Use fleet_manager-srv:tasks instead.")
  (tasks m))

(cl:ensure-generic-function 'assign_to_drone_id-val :lambda-list '(m))
(cl:defmethod assign_to_drone_id-val ((m <InjectTasks-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-srv:assign_to_drone_id-val is deprecated.  Use fleet_manager-srv:assign_to_drone_id instead.")
  (assign_to_drone_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InjectTasks-request>) ostream)
  "Serializes a message object of type '<InjectTasks-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tasks))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'assign_to_drone_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'assign_to_drone_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InjectTasks-request>) istream)
  "Deserializes a message object of type '<InjectTasks-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fleet_manager-msg:TaskStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'assign_to_drone_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'assign_to_drone_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InjectTasks-request>)))
  "Returns string type for a service object of type '<InjectTasks-request>"
  "fleet_manager/InjectTasksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InjectTasks-request)))
  "Returns string type for a service object of type 'InjectTasks-request"
  "fleet_manager/InjectTasksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InjectTasks-request>)))
  "Returns md5sum for a message object of type '<InjectTasks-request>"
  "72ac7b4be33eb67b134d27e55502c210")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InjectTasks-request)))
  "Returns md5sum for a message object of type 'InjectTasks-request"
  "72ac7b4be33eb67b134d27e55502c210")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InjectTasks-request>)))
  "Returns full string definition for message of type '<InjectTasks-request>"
  (cl:format cl:nil "fleet_manager/TaskStatus[] tasks~%string assign_to_drone_id~%~%================================================================================~%MSG: fleet_manager/TaskStatus~%# [修改] 在消息定义中添加常量~%# Task Status Constants~%uint8 PENDING=0~%uint8 IN_PROGRESS=1~%uint8 COMPLETED=2~%uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码~%~%# Message Fields~%int32 task_id~%nlp_drone_control/Action action_details~%uint8 status~%string assigned_drone_id~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InjectTasks-request)))
  "Returns full string definition for message of type 'InjectTasks-request"
  (cl:format cl:nil "fleet_manager/TaskStatus[] tasks~%string assign_to_drone_id~%~%================================================================================~%MSG: fleet_manager/TaskStatus~%# [修改] 在消息定义中添加常量~%# Task Status Constants~%uint8 PENDING=0~%uint8 IN_PROGRESS=1~%uint8 COMPLETED=2~%uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码~%~%# Message Fields~%int32 task_id~%nlp_drone_control/Action action_details~%uint8 status~%string assigned_drone_id~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InjectTasks-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'assign_to_drone_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InjectTasks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InjectTasks-request
    (cl:cons ':tasks (tasks msg))
    (cl:cons ':assign_to_drone_id (assign_to_drone_id msg))
))
;//! \htmlinclude InjectTasks-response.msg.html

(cl:defclass <InjectTasks-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InjectTasks-response (<InjectTasks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InjectTasks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InjectTasks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_manager-srv:<InjectTasks-response> is deprecated: use fleet_manager-srv:InjectTasks-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InjectTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-srv:success-val is deprecated.  Use fleet_manager-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InjectTasks-response>) ostream)
  "Serializes a message object of type '<InjectTasks-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InjectTasks-response>) istream)
  "Deserializes a message object of type '<InjectTasks-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InjectTasks-response>)))
  "Returns string type for a service object of type '<InjectTasks-response>"
  "fleet_manager/InjectTasksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InjectTasks-response)))
  "Returns string type for a service object of type 'InjectTasks-response"
  "fleet_manager/InjectTasksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InjectTasks-response>)))
  "Returns md5sum for a message object of type '<InjectTasks-response>"
  "72ac7b4be33eb67b134d27e55502c210")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InjectTasks-response)))
  "Returns md5sum for a message object of type 'InjectTasks-response"
  "72ac7b4be33eb67b134d27e55502c210")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InjectTasks-response>)))
  "Returns full string definition for message of type '<InjectTasks-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InjectTasks-response)))
  "Returns full string definition for message of type 'InjectTasks-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InjectTasks-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InjectTasks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InjectTasks-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InjectTasks)))
  'InjectTasks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InjectTasks)))
  'InjectTasks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InjectTasks)))
  "Returns string type for a service object of type '<InjectTasks>"
  "fleet_manager/InjectTasks")