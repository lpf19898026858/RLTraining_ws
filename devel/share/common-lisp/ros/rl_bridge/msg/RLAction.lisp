; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLAction.msg.html

(cl:defclass <RLAction> (roslisp-msg-protocol:ros-message)
  ((ax
    :reader ax
    :initarg :ax
    :type cl:float
    :initform 0.0)
   (ay
    :reader ay
    :initarg :ay
    :type cl:float
    :initform 0.0))
)

(cl:defclass RLAction (<RLAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLAction> is deprecated: use rl_bridge-msg:RLAction instead.")))

(cl:ensure-generic-function 'ax-val :lambda-list '(m))
(cl:defmethod ax-val ((m <RLAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:ax-val is deprecated.  Use rl_bridge-msg:ax instead.")
  (ax m))

(cl:ensure-generic-function 'ay-val :lambda-list '(m))
(cl:defmethod ay-val ((m <RLAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:ay-val is deprecated.  Use rl_bridge-msg:ay instead.")
  (ay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLAction>) ostream)
  "Serializes a message object of type '<RLAction>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ax))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ay))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLAction>) istream)
  "Deserializes a message object of type '<RLAction>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ax) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ay) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLAction>)))
  "Returns string type for a message object of type '<RLAction>"
  "rl_bridge/RLAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLAction)))
  "Returns string type for a message object of type 'RLAction"
  "rl_bridge/RLAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLAction>)))
  "Returns md5sum for a message object of type '<RLAction>"
  "4eb4e8d132b979f0df69e04e0a241d57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLAction)))
  "Returns md5sum for a message object of type 'RLAction"
  "4eb4e8d132b979f0df69e04e0a241d57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLAction>)))
  "Returns full string definition for message of type '<RLAction>"
  (cl:format cl:nil "float32 ax~%float32 ay~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLAction)))
  "Returns full string definition for message of type 'RLAction"
  (cl:format cl:nil "float32 ax~%float32 ay~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLAction>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLAction>))
  "Converts a ROS message object to a list"
  (cl:list 'RLAction
    (cl:cons ':ax (ax msg))
    (cl:cons ':ay (ay msg))
))
