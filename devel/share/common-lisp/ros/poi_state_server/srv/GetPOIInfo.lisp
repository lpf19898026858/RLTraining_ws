; Auto-generated. Do not edit!


(cl:in-package poi_state_server-srv)


;//! \htmlinclude GetPOIInfo-request.msg.html

(cl:defclass <GetPOIInfo-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetPOIInfo-request (<GetPOIInfo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPOIInfo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPOIInfo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-srv:<GetPOIInfo-request> is deprecated: use poi_state_server-srv:GetPOIInfo-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetPOIInfo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:name-val is deprecated.  Use poi_state_server-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPOIInfo-request>) ostream)
  "Serializes a message object of type '<GetPOIInfo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPOIInfo-request>) istream)
  "Deserializes a message object of type '<GetPOIInfo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPOIInfo-request>)))
  "Returns string type for a service object of type '<GetPOIInfo-request>"
  "poi_state_server/GetPOIInfoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPOIInfo-request)))
  "Returns string type for a service object of type 'GetPOIInfo-request"
  "poi_state_server/GetPOIInfoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPOIInfo-request>)))
  "Returns md5sum for a message object of type '<GetPOIInfo-request>"
  "c65c76ed817470389da7b2cfd1cdfbea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPOIInfo-request)))
  "Returns md5sum for a message object of type 'GetPOIInfo-request"
  "c65c76ed817470389da7b2cfd1cdfbea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPOIInfo-request>)))
  "Returns full string definition for message of type '<GetPOIInfo-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPOIInfo-request)))
  "Returns full string definition for message of type 'GetPOIInfo-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPOIInfo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPOIInfo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPOIInfo-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude GetPOIInfo-response.msg.html

(cl:defclass <GetPOIInfo-response> (roslisp-msg-protocol:ros-message)
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
   (info
    :reader info
    :initarg :info
    :type poi_state_server-msg:POIInfo
    :initform (cl:make-instance 'poi_state_server-msg:POIInfo)))
)

(cl:defclass GetPOIInfo-response (<GetPOIInfo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPOIInfo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPOIInfo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-srv:<GetPOIInfo-response> is deprecated: use poi_state_server-srv:GetPOIInfo-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetPOIInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:success-val is deprecated.  Use poi_state_server-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GetPOIInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:message-val is deprecated.  Use poi_state_server-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <GetPOIInfo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:info-val is deprecated.  Use poi_state_server-srv:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPOIInfo-response>) ostream)
  "Serializes a message object of type '<GetPOIInfo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'info) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPOIInfo-response>) istream)
  "Deserializes a message object of type '<GetPOIInfo-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'info) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPOIInfo-response>)))
  "Returns string type for a service object of type '<GetPOIInfo-response>"
  "poi_state_server/GetPOIInfoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPOIInfo-response)))
  "Returns string type for a service object of type 'GetPOIInfo-response"
  "poi_state_server/GetPOIInfoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPOIInfo-response>)))
  "Returns md5sum for a message object of type '<GetPOIInfo-response>"
  "c65c76ed817470389da7b2cfd1cdfbea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPOIInfo-response)))
  "Returns md5sum for a message object of type 'GetPOIInfo-response"
  "c65c76ed817470389da7b2cfd1cdfbea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPOIInfo-response>)))
  "Returns full string definition for message of type '<GetPOIInfo-response>"
  (cl:format cl:nil "bool success~%string message~%poi_state_server/POIInfo info~%~%~%~%================================================================================~%MSG: poi_state_server/POIInfo~%# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPOIInfo-response)))
  "Returns full string definition for message of type 'GetPOIInfo-response"
  (cl:format cl:nil "bool success~%string message~%poi_state_server/POIInfo info~%~%~%~%================================================================================~%MSG: poi_state_server/POIInfo~%# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPOIInfo-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPOIInfo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPOIInfo-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':info (info msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPOIInfo)))
  'GetPOIInfo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPOIInfo)))
  'GetPOIInfo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPOIInfo)))
  "Returns string type for a service object of type '<GetPOIInfo>"
  "poi_state_server/GetPOIInfo")