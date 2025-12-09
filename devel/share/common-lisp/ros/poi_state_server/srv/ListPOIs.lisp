; Auto-generated. Do not edit!


(cl:in-package poi_state_server-srv)


;//! \htmlinclude ListPOIs-request.msg.html

(cl:defclass <ListPOIs-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListPOIs-request (<ListPOIs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListPOIs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListPOIs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-srv:<ListPOIs-request> is deprecated: use poi_state_server-srv:ListPOIs-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListPOIs-request>) ostream)
  "Serializes a message object of type '<ListPOIs-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListPOIs-request>) istream)
  "Deserializes a message object of type '<ListPOIs-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListPOIs-request>)))
  "Returns string type for a service object of type '<ListPOIs-request>"
  "poi_state_server/ListPOIsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListPOIs-request)))
  "Returns string type for a service object of type 'ListPOIs-request"
  "poi_state_server/ListPOIsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListPOIs-request>)))
  "Returns md5sum for a message object of type '<ListPOIs-request>"
  "5729bcda884e859dbb5f9dba61bed21b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListPOIs-request)))
  "Returns md5sum for a message object of type 'ListPOIs-request"
  "5729bcda884e859dbb5f9dba61bed21b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListPOIs-request>)))
  "Returns full string definition for message of type '<ListPOIs-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListPOIs-request)))
  "Returns full string definition for message of type 'ListPOIs-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListPOIs-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListPOIs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListPOIs-request
))
;//! \htmlinclude ListPOIs-response.msg.html

(cl:defclass <ListPOIs-response> (roslisp-msg-protocol:ros-message)
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
   (pois
    :reader pois
    :initarg :pois
    :type (cl:vector poi_state_server-msg:POIInfo)
   :initform (cl:make-array 0 :element-type 'poi_state_server-msg:POIInfo :initial-element (cl:make-instance 'poi_state_server-msg:POIInfo))))
)

(cl:defclass ListPOIs-response (<ListPOIs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListPOIs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListPOIs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-srv:<ListPOIs-response> is deprecated: use poi_state_server-srv:ListPOIs-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ListPOIs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:success-val is deprecated.  Use poi_state_server-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ListPOIs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:message-val is deprecated.  Use poi_state_server-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'pois-val :lambda-list '(m))
(cl:defmethod pois-val ((m <ListPOIs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-srv:pois-val is deprecated.  Use poi_state_server-srv:pois instead.")
  (pois m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListPOIs-response>) ostream)
  "Serializes a message object of type '<ListPOIs-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pois))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pois))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListPOIs-response>) istream)
  "Deserializes a message object of type '<ListPOIs-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pois) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pois)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'poi_state_server-msg:POIInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListPOIs-response>)))
  "Returns string type for a service object of type '<ListPOIs-response>"
  "poi_state_server/ListPOIsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListPOIs-response)))
  "Returns string type for a service object of type 'ListPOIs-response"
  "poi_state_server/ListPOIsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListPOIs-response>)))
  "Returns md5sum for a message object of type '<ListPOIs-response>"
  "5729bcda884e859dbb5f9dba61bed21b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListPOIs-response)))
  "Returns md5sum for a message object of type 'ListPOIs-response"
  "5729bcda884e859dbb5f9dba61bed21b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListPOIs-response>)))
  "Returns full string definition for message of type '<ListPOIs-response>"
  (cl:format cl:nil "bool success~%string message~%poi_state_server/POIInfo[] pois~%~%~%~%================================================================================~%MSG: poi_state_server/POIInfo~%# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListPOIs-response)))
  "Returns full string definition for message of type 'ListPOIs-response"
  (cl:format cl:nil "bool success~%string message~%poi_state_server/POIInfo[] pois~%~%~%~%================================================================================~%MSG: poi_state_server/POIInfo~%# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListPOIs-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pois) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListPOIs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListPOIs-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':pois (pois msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListPOIs)))
  'ListPOIs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListPOIs)))
  'ListPOIs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListPOIs)))
  "Returns string type for a service object of type '<ListPOIs>"
  "poi_state_server/ListPOIs")