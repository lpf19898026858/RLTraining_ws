; Auto-generated. Do not edit!


(cl:in-package fleet_manager-srv)


;//! \htmlinclude QueryFleetAndTasks-request.msg.html

(cl:defclass <QueryFleetAndTasks-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass QueryFleetAndTasks-request (<QueryFleetAndTasks-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryFleetAndTasks-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryFleetAndTasks-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_manager-srv:<QueryFleetAndTasks-request> is deprecated: use fleet_manager-srv:QueryFleetAndTasks-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryFleetAndTasks-request>) ostream)
  "Serializes a message object of type '<QueryFleetAndTasks-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryFleetAndTasks-request>) istream)
  "Deserializes a message object of type '<QueryFleetAndTasks-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryFleetAndTasks-request>)))
  "Returns string type for a service object of type '<QueryFleetAndTasks-request>"
  "fleet_manager/QueryFleetAndTasksRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFleetAndTasks-request)))
  "Returns string type for a service object of type 'QueryFleetAndTasks-request"
  "fleet_manager/QueryFleetAndTasksRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryFleetAndTasks-request>)))
  "Returns md5sum for a message object of type '<QueryFleetAndTasks-request>"
  "a9cee634e5450b9a2e85c18da8e84744")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryFleetAndTasks-request)))
  "Returns md5sum for a message object of type 'QueryFleetAndTasks-request"
  "a9cee634e5450b9a2e85c18da8e84744")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryFleetAndTasks-request>)))
  "Returns full string definition for message of type '<QueryFleetAndTasks-request>"
  (cl:format cl:nil "# 请求部分为空~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryFleetAndTasks-request)))
  "Returns full string definition for message of type 'QueryFleetAndTasks-request"
  (cl:format cl:nil "# 请求部分为空~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryFleetAndTasks-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryFleetAndTasks-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryFleetAndTasks-request
))
;//! \htmlinclude QueryFleetAndTasks-response.msg.html

(cl:defclass <QueryFleetAndTasks-response> (roslisp-msg-protocol:ros-message)
  ((drones
    :reader drones
    :initarg :drones
    :type (cl:vector fleet_manager-msg:DroneStatus)
   :initform (cl:make-array 0 :element-type 'fleet_manager-msg:DroneStatus :initial-element (cl:make-instance 'fleet_manager-msg:DroneStatus)))
   (tasks
    :reader tasks
    :initarg :tasks
    :type (cl:vector fleet_manager-msg:TaskStatus)
   :initform (cl:make-array 0 :element-type 'fleet_manager-msg:TaskStatus :initial-element (cl:make-instance 'fleet_manager-msg:TaskStatus))))
)

(cl:defclass QueryFleetAndTasks-response (<QueryFleetAndTasks-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryFleetAndTasks-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryFleetAndTasks-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_manager-srv:<QueryFleetAndTasks-response> is deprecated: use fleet_manager-srv:QueryFleetAndTasks-response instead.")))

(cl:ensure-generic-function 'drones-val :lambda-list '(m))
(cl:defmethod drones-val ((m <QueryFleetAndTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-srv:drones-val is deprecated.  Use fleet_manager-srv:drones instead.")
  (drones m))

(cl:ensure-generic-function 'tasks-val :lambda-list '(m))
(cl:defmethod tasks-val ((m <QueryFleetAndTasks-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-srv:tasks-val is deprecated.  Use fleet_manager-srv:tasks instead.")
  (tasks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryFleetAndTasks-response>) ostream)
  "Serializes a message object of type '<QueryFleetAndTasks-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'drones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'drones))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tasks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryFleetAndTasks-response>) istream)
  "Deserializes a message object of type '<QueryFleetAndTasks-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'drones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'drones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fleet_manager-msg:DroneStatus))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryFleetAndTasks-response>)))
  "Returns string type for a service object of type '<QueryFleetAndTasks-response>"
  "fleet_manager/QueryFleetAndTasksResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFleetAndTasks-response)))
  "Returns string type for a service object of type 'QueryFleetAndTasks-response"
  "fleet_manager/QueryFleetAndTasksResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryFleetAndTasks-response>)))
  "Returns md5sum for a message object of type '<QueryFleetAndTasks-response>"
  "a9cee634e5450b9a2e85c18da8e84744")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryFleetAndTasks-response)))
  "Returns md5sum for a message object of type 'QueryFleetAndTasks-response"
  "a9cee634e5450b9a2e85c18da8e84744")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryFleetAndTasks-response>)))
  "Returns full string definition for message of type '<QueryFleetAndTasks-response>"
  (cl:format cl:nil "# 响应部分~%fleet_manager/DroneStatus[] drones~%fleet_manager/TaskStatus[] tasks~%~%~%================================================================================~%MSG: fleet_manager/DroneStatus~%# [修改] 在消息定义中添加常量~%# Drone Status Constants~%uint8 UNKNOWN=0~%uint8 ACTIVE=1~%uint8 NEEDS_BACKUP=2~%uint8 OFFLINE=3~%~%# Message Fields~%string drone_id~%uint8 status~%int32 last_completed_task_index~%geometry_msgs/Pose last_known_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: fleet_manager/TaskStatus~%# [修改] 在消息定义中添加常量~%# Task Status Constants~%uint8 PENDING=0~%uint8 IN_PROGRESS=1~%uint8 COMPLETED=2~%uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码~%~%# Message Fields~%int32 task_id~%nlp_drone_control/Action action_details~%uint8 status~%string assigned_drone_id~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryFleetAndTasks-response)))
  "Returns full string definition for message of type 'QueryFleetAndTasks-response"
  (cl:format cl:nil "# 响应部分~%fleet_manager/DroneStatus[] drones~%fleet_manager/TaskStatus[] tasks~%~%~%================================================================================~%MSG: fleet_manager/DroneStatus~%# [修改] 在消息定义中添加常量~%# Drone Status Constants~%uint8 UNKNOWN=0~%uint8 ACTIVE=1~%uint8 NEEDS_BACKUP=2~%uint8 OFFLINE=3~%~%# Message Fields~%string drone_id~%uint8 status~%int32 last_completed_task_index~%geometry_msgs/Pose last_known_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: fleet_manager/TaskStatus~%# [修改] 在消息定义中添加常量~%# Task Status Constants~%uint8 PENDING=0~%uint8 IN_PROGRESS=1~%uint8 COMPLETED=2~%uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码~%~%# Message Fields~%int32 task_id~%nlp_drone_control/Action action_details~%uint8 status~%string assigned_drone_id~%~%================================================================================~%MSG: nlp_drone_control/Action~%# Represents a single tool call produced by the LLM~%string function_name       # e.g. \"takeoff\"~%string arguments_json      # raw JSON string of arguments~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryFleetAndTasks-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'drones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryFleetAndTasks-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryFleetAndTasks-response
    (cl:cons ':drones (drones msg))
    (cl:cons ':tasks (tasks msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryFleetAndTasks)))
  'QueryFleetAndTasks-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryFleetAndTasks)))
  'QueryFleetAndTasks-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFleetAndTasks)))
  "Returns string type for a service object of type '<QueryFleetAndTasks>"
  "fleet_manager/QueryFleetAndTasks")