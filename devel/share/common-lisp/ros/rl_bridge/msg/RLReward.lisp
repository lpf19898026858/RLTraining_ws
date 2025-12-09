; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLReward.msg.html

(cl:defclass <RLReward> (roslisp-msg-protocol:ros-message)
  ((reward
    :reader reward
    :initarg :reward
    :type cl:float
    :initform 0.0))
)

(cl:defclass RLReward (<RLReward>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLReward>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLReward)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLReward> is deprecated: use rl_bridge-msg:RLReward instead.")))

(cl:ensure-generic-function 'reward-val :lambda-list '(m))
(cl:defmethod reward-val ((m <RLReward>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:reward-val is deprecated.  Use rl_bridge-msg:reward instead.")
  (reward m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLReward>) ostream)
  "Serializes a message object of type '<RLReward>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLReward>) istream)
  "Deserializes a message object of type '<RLReward>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reward) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLReward>)))
  "Returns string type for a message object of type '<RLReward>"
  "rl_bridge/RLReward")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLReward)))
  "Returns string type for a message object of type 'RLReward"
  "rl_bridge/RLReward")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLReward>)))
  "Returns md5sum for a message object of type '<RLReward>"
  "1c5a2bbc277e822b80ec2fb352dd1efe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLReward)))
  "Returns md5sum for a message object of type 'RLReward"
  "1c5a2bbc277e822b80ec2fb352dd1efe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLReward>)))
  "Returns full string definition for message of type '<RLReward>"
  (cl:format cl:nil "float32 reward~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLReward)))
  "Returns full string definition for message of type 'RLReward"
  (cl:format cl:nil "float32 reward~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLReward>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLReward>))
  "Converts a ROS message object to a list"
  (cl:list 'RLReward
    (cl:cons ':reward (reward msg))
))
