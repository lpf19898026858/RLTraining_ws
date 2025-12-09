; Auto-generated. Do not edit!


(cl:in-package custom_msgs-srv)


;//! \htmlinclude UpdateCurriculum-request.msg.html

(cl:defclass <UpdateCurriculum-request> (roslisp-msg-protocol:ros-message)
  ((spawn_range_x
    :reader spawn_range_x
    :initarg :spawn_range_x
    :type cl:float
    :initform 0.0)
   (spawn_range_z
    :reader spawn_range_z
    :initarg :spawn_range_z
    :type cl:float
    :initform 0.0)
   (local_target_range
    :reader local_target_range
    :initarg :local_target_range
    :type cl:float
    :initform 0.0))
)

(cl:defclass UpdateCurriculum-request (<UpdateCurriculum-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateCurriculum-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateCurriculum-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<UpdateCurriculum-request> is deprecated: use custom_msgs-srv:UpdateCurriculum-request instead.")))

(cl:ensure-generic-function 'spawn_range_x-val :lambda-list '(m))
(cl:defmethod spawn_range_x-val ((m <UpdateCurriculum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:spawn_range_x-val is deprecated.  Use custom_msgs-srv:spawn_range_x instead.")
  (spawn_range_x m))

(cl:ensure-generic-function 'spawn_range_z-val :lambda-list '(m))
(cl:defmethod spawn_range_z-val ((m <UpdateCurriculum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:spawn_range_z-val is deprecated.  Use custom_msgs-srv:spawn_range_z instead.")
  (spawn_range_z m))

(cl:ensure-generic-function 'local_target_range-val :lambda-list '(m))
(cl:defmethod local_target_range-val ((m <UpdateCurriculum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:local_target_range-val is deprecated.  Use custom_msgs-srv:local_target_range instead.")
  (local_target_range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateCurriculum-request>) ostream)
  "Serializes a message object of type '<UpdateCurriculum-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'spawn_range_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'spawn_range_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'local_target_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateCurriculum-request>) istream)
  "Deserializes a message object of type '<UpdateCurriculum-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'spawn_range_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'spawn_range_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'local_target_range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateCurriculum-request>)))
  "Returns string type for a service object of type '<UpdateCurriculum-request>"
  "custom_msgs/UpdateCurriculumRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateCurriculum-request)))
  "Returns string type for a service object of type 'UpdateCurriculum-request"
  "custom_msgs/UpdateCurriculumRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateCurriculum-request>)))
  "Returns md5sum for a message object of type '<UpdateCurriculum-request>"
  "3b2cd8798f0550ceaa4d103980369410")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateCurriculum-request)))
  "Returns md5sum for a message object of type 'UpdateCurriculum-request"
  "3b2cd8798f0550ceaa4d103980369410")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateCurriculum-request>)))
  "Returns full string definition for message of type '<UpdateCurriculum-request>"
  (cl:format cl:nil "float32 spawn_range_x~%float32 spawn_range_z~%float32 local_target_range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateCurriculum-request)))
  "Returns full string definition for message of type 'UpdateCurriculum-request"
  (cl:format cl:nil "float32 spawn_range_x~%float32 spawn_range_z~%float32 local_target_range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateCurriculum-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateCurriculum-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateCurriculum-request
    (cl:cons ':spawn_range_x (spawn_range_x msg))
    (cl:cons ':spawn_range_z (spawn_range_z msg))
    (cl:cons ':local_target_range (local_target_range msg))
))
;//! \htmlinclude UpdateCurriculum-response.msg.html

(cl:defclass <UpdateCurriculum-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateCurriculum-response (<UpdateCurriculum-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateCurriculum-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateCurriculum-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<UpdateCurriculum-response> is deprecated: use custom_msgs-srv:UpdateCurriculum-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateCurriculum-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:success-val is deprecated.  Use custom_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateCurriculum-response>) ostream)
  "Serializes a message object of type '<UpdateCurriculum-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateCurriculum-response>) istream)
  "Deserializes a message object of type '<UpdateCurriculum-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateCurriculum-response>)))
  "Returns string type for a service object of type '<UpdateCurriculum-response>"
  "custom_msgs/UpdateCurriculumResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateCurriculum-response)))
  "Returns string type for a service object of type 'UpdateCurriculum-response"
  "custom_msgs/UpdateCurriculumResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateCurriculum-response>)))
  "Returns md5sum for a message object of type '<UpdateCurriculum-response>"
  "3b2cd8798f0550ceaa4d103980369410")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateCurriculum-response)))
  "Returns md5sum for a message object of type 'UpdateCurriculum-response"
  "3b2cd8798f0550ceaa4d103980369410")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateCurriculum-response>)))
  "Returns full string definition for message of type '<UpdateCurriculum-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateCurriculum-response)))
  "Returns full string definition for message of type 'UpdateCurriculum-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateCurriculum-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateCurriculum-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateCurriculum-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateCurriculum)))
  'UpdateCurriculum-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateCurriculum)))
  'UpdateCurriculum-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateCurriculum)))
  "Returns string type for a service object of type '<UpdateCurriculum>"
  "custom_msgs/UpdateCurriculum")