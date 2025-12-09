; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLReset.msg.html

(cl:defclass <RLReset> (roslisp-msg-protocol:ros-message)
  ((reset
    :reader reset
    :initarg :reset
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RLReset (<RLReset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLReset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLReset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLReset> is deprecated: use rl_bridge-msg:RLReset instead.")))

(cl:ensure-generic-function 'reset-val :lambda-list '(m))
(cl:defmethod reset-val ((m <RLReset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:reset-val is deprecated.  Use rl_bridge-msg:reset instead.")
  (reset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLReset>) ostream)
  "Serializes a message object of type '<RLReset>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLReset>) istream)
  "Deserializes a message object of type '<RLReset>"
    (cl:setf (cl:slot-value msg 'reset) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLReset>)))
  "Returns string type for a message object of type '<RLReset>"
  "rl_bridge/RLReset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLReset)))
  "Returns string type for a message object of type 'RLReset"
  "rl_bridge/RLReset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLReset>)))
  "Returns md5sum for a message object of type '<RLReset>"
  "ba4b0b221fb425ac5eaf73f71ae34971")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLReset)))
  "Returns md5sum for a message object of type 'RLReset"
  "ba4b0b221fb425ac5eaf73f71ae34971")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLReset>)))
  "Returns full string definition for message of type '<RLReset>"
  (cl:format cl:nil "bool reset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLReset)))
  "Returns full string definition for message of type 'RLReset"
  (cl:format cl:nil "bool reset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLReset>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLReset>))
  "Converts a ROS message object to a list"
  (cl:list 'RLReset
    (cl:cons ':reset (reset msg))
))
