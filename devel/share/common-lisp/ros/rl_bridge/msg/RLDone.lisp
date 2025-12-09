; Auto-generated. Do not edit!


(cl:in-package rl_bridge-msg)


;//! \htmlinclude RLDone.msg.html

(cl:defclass <RLDone> (roslisp-msg-protocol:ros-message)
  ((done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RLDone (<RLDone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RLDone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RLDone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rl_bridge-msg:<RLDone> is deprecated: use rl_bridge-msg:RLDone instead.")))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <RLDone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rl_bridge-msg:done-val is deprecated.  Use rl_bridge-msg:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RLDone>) ostream)
  "Serializes a message object of type '<RLDone>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RLDone>) istream)
  "Deserializes a message object of type '<RLDone>"
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RLDone>)))
  "Returns string type for a message object of type '<RLDone>"
  "rl_bridge/RLDone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RLDone)))
  "Returns string type for a message object of type 'RLDone"
  "rl_bridge/RLDone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RLDone>)))
  "Returns md5sum for a message object of type '<RLDone>"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RLDone)))
  "Returns md5sum for a message object of type 'RLDone"
  "89bb254424e4cffedbf494e7b0ddbfea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RLDone>)))
  "Returns full string definition for message of type '<RLDone>"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RLDone)))
  "Returns full string definition for message of type 'RLDone"
  (cl:format cl:nil "bool done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RLDone>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RLDone>))
  "Converts a ROS message object to a list"
  (cl:list 'RLDone
    (cl:cons ':done (done msg))
))
