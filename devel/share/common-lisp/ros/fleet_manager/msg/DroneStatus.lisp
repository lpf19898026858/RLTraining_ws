; Auto-generated. Do not edit!


(cl:in-package fleet_manager-msg)


;//! \htmlinclude DroneStatus.msg.html

(cl:defclass <DroneStatus> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (last_completed_task_index
    :reader last_completed_task_index
    :initarg :last_completed_task_index
    :type cl:integer
    :initform 0)
   (last_known_pose
    :reader last_known_pose
    :initarg :last_known_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass DroneStatus (<DroneStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_manager-msg:<DroneStatus> is deprecated: use fleet_manager-msg:DroneStatus instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <DroneStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-msg:drone_id-val is deprecated.  Use fleet_manager-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <DroneStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-msg:status-val is deprecated.  Use fleet_manager-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'last_completed_task_index-val :lambda-list '(m))
(cl:defmethod last_completed_task_index-val ((m <DroneStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-msg:last_completed_task_index-val is deprecated.  Use fleet_manager-msg:last_completed_task_index instead.")
  (last_completed_task_index m))

(cl:ensure-generic-function 'last_known_pose-val :lambda-list '(m))
(cl:defmethod last_known_pose-val ((m <DroneStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_manager-msg:last_known_pose-val is deprecated.  Use fleet_manager-msg:last_known_pose instead.")
  (last_known_pose m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DroneStatus>)))
    "Constants for message type '<DroneStatus>"
  '((:UNKNOWN . 0)
    (:ACTIVE . 1)
    (:NEEDS_BACKUP . 2)
    (:OFFLINE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DroneStatus)))
    "Constants for message type 'DroneStatus"
  '((:UNKNOWN . 0)
    (:ACTIVE . 1)
    (:NEEDS_BACKUP . 2)
    (:OFFLINE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneStatus>) ostream)
  "Serializes a message object of type '<DroneStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'drone_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'drone_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'last_completed_task_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'last_known_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneStatus>) istream)
  "Deserializes a message object of type '<DroneStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'drone_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_completed_task_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'last_known_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneStatus>)))
  "Returns string type for a message object of type '<DroneStatus>"
  "fleet_manager/DroneStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneStatus)))
  "Returns string type for a message object of type 'DroneStatus"
  "fleet_manager/DroneStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneStatus>)))
  "Returns md5sum for a message object of type '<DroneStatus>"
  "4399a33682bd42bcbd894637e74a998e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneStatus)))
  "Returns md5sum for a message object of type 'DroneStatus"
  "4399a33682bd42bcbd894637e74a998e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneStatus>)))
  "Returns full string definition for message of type '<DroneStatus>"
  (cl:format cl:nil "# [修改] 在消息定义中添加常量~%# Drone Status Constants~%uint8 UNKNOWN=0~%uint8 ACTIVE=1~%uint8 NEEDS_BACKUP=2~%uint8 OFFLINE=3~%~%# Message Fields~%string drone_id~%uint8 status~%int32 last_completed_task_index~%geometry_msgs/Pose last_known_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneStatus)))
  "Returns full string definition for message of type 'DroneStatus"
  (cl:format cl:nil "# [修改] 在消息定义中添加常量~%# Drone Status Constants~%uint8 UNKNOWN=0~%uint8 ACTIVE=1~%uint8 NEEDS_BACKUP=2~%uint8 OFFLINE=3~%~%# Message Fields~%string drone_id~%uint8 status~%int32 last_completed_task_index~%geometry_msgs/Pose last_known_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'drone_id))
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'last_known_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneStatus
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':status (status msg))
    (cl:cons ':last_completed_task_index (last_completed_task_index msg))
    (cl:cons ':last_known_pose (last_known_pose msg))
))
