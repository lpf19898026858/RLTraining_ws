; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLObservation.msg.html

(cl:defclass <RLObservation> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0))
)

(cl:defclass RLObservation (<RLObservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLObservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLObservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLObservation> is deprecated: use rl_bridge-msg:RLObservation instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RLObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:x-val is deprecated.  Use rl_bridge-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RLObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:y-val is deprecated.  Use rl_bridge-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <RLObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:vx-val is deprecated.  Use rl_bridge-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <RLObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:vy-val is deprecated.  Use rl_bridge-msg:vy instead.")
  (vy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLObservation>) ostream)
  "Serializes a message object of type '<RLObservation>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLObservation>) istream)
  "Deserializes a message object of type '<RLObservation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLObservation>)))
  "Returns string type for a message object of type '<RLObservation>"
  "rl_bridge/RLObservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLObservation)))
  "Returns string type for a message object of type 'RLObservation"
  "rl_bridge/RLObservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLObservation>)))
  "Returns md5sum for a message object of type '<RLObservation>"
  "2af08925b9da4c99c24669eae33385e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLObservation)))
  "Returns md5sum for a message object of type 'RLObservation"
  "2af08925b9da4c99c24669eae33385e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLObservation>)))
  "Returns full string definition for message of type '<RLObservation>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 vx~%float32 vy~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLObservation)))
  "Returns full string definition for message of type 'RLObservation"
  (cl:format cl:nil "float32 x~%float32 y~%float32 vx~%float32 vy~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLObservation>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLObservation>))
  "Converts a ROS message object to a list"
  (cl:list 'RLObservation
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
))
