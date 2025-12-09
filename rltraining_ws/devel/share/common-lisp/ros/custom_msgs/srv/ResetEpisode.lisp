; Auto-generated. Do not edit!


(cl:in-package custom_msgs-srv)


;//! \htmlinclude ResetEpisode-request.msg.html

(cl:defclass <ResetEpisode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetEpisode-request (<ResetEpisode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetEpisode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetEpisode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<ResetEpisode-request> is deprecated: use custom_msgs-srv:ResetEpisode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetEpisode-request>) ostream)
  "Serializes a message object of type '<ResetEpisode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetEpisode-request>) istream)
  "Deserializes a message object of type '<ResetEpisode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetEpisode-request>)))
  "Returns string type for a service object of type '<ResetEpisode-request>"
  "custom_msgs/ResetEpisodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEpisode-request)))
  "Returns string type for a service object of type 'ResetEpisode-request"
  "custom_msgs/ResetEpisodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetEpisode-request>)))
  "Returns md5sum for a message object of type '<ResetEpisode-request>"
  "e566690f9c9e0029132e05fa58e0f836")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetEpisode-request)))
  "Returns md5sum for a message object of type 'ResetEpisode-request"
  "e566690f9c9e0029132e05fa58e0f836")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetEpisode-request>)))
  "Returns full string definition for message of type '<ResetEpisode-request>"
  (cl:format cl:nil "# custom_msgs/srv/ResetEpisode.srv~%~%# --- Request ---~%# 请求可以是空的，因为我们只是想触发一个重置~%# 可以加一个bool来支持课程学习，但现在先保持简单~%# bool use_curriculum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetEpisode-request)))
  "Returns full string definition for message of type 'ResetEpisode-request"
  (cl:format cl:nil "# custom_msgs/srv/ResetEpisode.srv~%~%# --- Request ---~%# 请求可以是空的，因为我们只是想触发一个重置~%# 可以加一个bool来支持课程学习，但现在先保持简单~%# bool use_curriculum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetEpisode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetEpisode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetEpisode-request
))
;//! \htmlinclude ResetEpisode-response.msg.html

(cl:defclass <ResetEpisode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (start_pose
    :reader start_pose
    :initarg :start_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (goal_position
    :reader goal_position
    :initarg :goal_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ResetEpisode-response (<ResetEpisode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetEpisode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetEpisode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<ResetEpisode-response> is deprecated: use custom_msgs-srv:ResetEpisode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:success-val is deprecated.  Use custom_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ResetEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:message-val is deprecated.  Use custom_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'start_pose-val :lambda-list '(m))
(cl:defmethod start_pose-val ((m <ResetEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:start_pose-val is deprecated.  Use custom_msgs-srv:start_pose instead.")
  (start_pose m))

(cl:ensure-generic-function 'goal_position-val :lambda-list '(m))
(cl:defmethod goal_position-val ((m <ResetEpisode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:goal_position-val is deprecated.  Use custom_msgs-srv:goal_position instead.")
  (goal_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetEpisode-response>) ostream)
  "Serializes a message object of type '<ResetEpisode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetEpisode-response>) istream)
  "Deserializes a message object of type '<ResetEpisode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetEpisode-response>)))
  "Returns string type for a service object of type '<ResetEpisode-response>"
  "custom_msgs/ResetEpisodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEpisode-response)))
  "Returns string type for a service object of type 'ResetEpisode-response"
  "custom_msgs/ResetEpisodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetEpisode-response>)))
  "Returns md5sum for a message object of type '<ResetEpisode-response>"
  "e566690f9c9e0029132e05fa58e0f836")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetEpisode-response)))
  "Returns md5sum for a message object of type 'ResetEpisode-response"
  "e566690f9c9e0029132e05fa58e0f836")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetEpisode-response>)))
  "Returns full string definition for message of type '<ResetEpisode-response>"
  (cl:format cl:nil "~%# --- Response ---~%# Unity找到的安全点将在这里返回~%bool success          # 标记重置是否成功~%string message        # 附带信息 (例如，失败原因)~%geometry_msgs/Pose start_pose~%geometry_msgs/Point goal_position~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetEpisode-response)))
  "Returns full string definition for message of type 'ResetEpisode-response"
  (cl:format cl:nil "~%# --- Response ---~%# Unity找到的安全点将在这里返回~%bool success          # 标记重置是否成功~%string message        # 附带信息 (例如，失败原因)~%geometry_msgs/Pose start_pose~%geometry_msgs/Point goal_position~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetEpisode-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetEpisode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetEpisode-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':start_pose (start_pose msg))
    (cl:cons ':goal_position (goal_position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetEpisode)))
  'ResetEpisode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetEpisode)))
  'ResetEpisode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetEpisode)))
  "Returns string type for a service object of type '<ResetEpisode>"
  "custom_msgs/ResetEpisode")