; Auto-generated. Do not edit!


(cl:in-package poi_state_server-msg)


;//! \htmlinclude NamedPoint.msg.html

(cl:defclass <NamedPoint> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass NamedPoint (<NamedPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NamedPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NamedPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-msg:<NamedPoint> is deprecated: use poi_state_server-msg:NamedPoint instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <NamedPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:name-val is deprecated.  Use poi_state_server-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <NamedPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:point-val is deprecated.  Use poi_state_server-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NamedPoint>) ostream)
  "Serializes a message object of type '<NamedPoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NamedPoint>) istream)
  "Deserializes a message object of type '<NamedPoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NamedPoint>)))
  "Returns string type for a message object of type '<NamedPoint>"
  "poi_state_server/NamedPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NamedPoint)))
  "Returns string type for a message object of type 'NamedPoint"
  "poi_state_server/NamedPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NamedPoint>)))
  "Returns md5sum for a message object of type '<NamedPoint>"
  "679c7c057ebc773242bf1f625c1c9996")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NamedPoint)))
  "Returns md5sum for a message object of type 'NamedPoint"
  "679c7c057ebc773242bf1f625c1c9996")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NamedPoint>)))
  "Returns full string definition for message of type '<NamedPoint>"
  (cl:format cl:nil "string name~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NamedPoint)))
  "Returns full string definition for message of type 'NamedPoint"
  (cl:format cl:nil "string name~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NamedPoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NamedPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'NamedPoint
    (cl:cons ':name (name msg))
    (cl:cons ':point (point msg))
))
