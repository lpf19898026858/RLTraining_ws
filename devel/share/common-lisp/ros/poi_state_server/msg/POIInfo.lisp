; Auto-generated. Do not edit!


(cl:in-package poi_state_server-msg)


;//! \htmlinclude POIInfo.msg.html

(cl:defclass <POIInfo> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (candidate_points
    :reader candidate_points
    :initarg :candidate_points
    :type (cl:vector poi_state_server-msg:NamedPoint)
   :initform (cl:make-array 0 :element-type 'poi_state_server-msg:NamedPoint :initial-element (cl:make-instance 'poi_state_server-msg:NamedPoint)))
   (boundary_min
    :reader boundary_min
    :initarg :boundary_min
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (boundary_max
    :reader boundary_max
    :initarg :boundary_max
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (has_boundary
    :reader has_boundary
    :initarg :has_boundary
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass POIInfo (<POIInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <POIInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'POIInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name poi_state_server-msg:<POIInfo> is deprecated: use poi_state_server-msg:POIInfo instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:name-val is deprecated.  Use poi_state_server-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:type-val is deprecated.  Use poi_state_server-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:description-val is deprecated.  Use poi_state_server-msg:description instead.")
  (description m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:position-val is deprecated.  Use poi_state_server-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'candidate_points-val :lambda-list '(m))
(cl:defmethod candidate_points-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:candidate_points-val is deprecated.  Use poi_state_server-msg:candidate_points instead.")
  (candidate_points m))

(cl:ensure-generic-function 'boundary_min-val :lambda-list '(m))
(cl:defmethod boundary_min-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:boundary_min-val is deprecated.  Use poi_state_server-msg:boundary_min instead.")
  (boundary_min m))

(cl:ensure-generic-function 'boundary_max-val :lambda-list '(m))
(cl:defmethod boundary_max-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:boundary_max-val is deprecated.  Use poi_state_server-msg:boundary_max instead.")
  (boundary_max m))

(cl:ensure-generic-function 'has_boundary-val :lambda-list '(m))
(cl:defmethod has_boundary-val ((m <POIInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader poi_state_server-msg:has_boundary-val is deprecated.  Use poi_state_server-msg:has_boundary instead.")
  (has_boundary m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <POIInfo>) ostream)
  "Serializes a message object of type '<POIInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'candidate_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'candidate_points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'boundary_min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'boundary_max) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_boundary) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <POIInfo>) istream)
  "Deserializes a message object of type '<POIInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'candidate_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'candidate_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'poi_state_server-msg:NamedPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'boundary_min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'boundary_max) istream)
    (cl:setf (cl:slot-value msg 'has_boundary) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<POIInfo>)))
  "Returns string type for a message object of type '<POIInfo>"
  "poi_state_server/POIInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'POIInfo)))
  "Returns string type for a message object of type 'POIInfo"
  "poi_state_server/POIInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<POIInfo>)))
  "Returns md5sum for a message object of type '<POIInfo>"
  "4b1a999c05b4e892b655e03546fef00e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'POIInfo)))
  "Returns md5sum for a message object of type 'POIInfo"
  "4b1a999c05b4e892b655e03546fef00e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<POIInfo>)))
  "Returns full string definition for message of type '<POIInfo>"
  (cl:format cl:nil "# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'POIInfo)))
  "Returns full string definition for message of type 'POIInfo"
  (cl:format cl:nil "# POIInfo.msg~%string name~%string type~%string description~%geometry_msgs/Point position~%poi_state_server/NamedPoint[] candidate_points~%geometry_msgs/Point boundary_min~%geometry_msgs/Point boundary_max~%bool has_boundary~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: poi_state_server/NamedPoint~%string name~%geometry_msgs/Point point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <POIInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'description))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'candidate_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'boundary_min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'boundary_max))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <POIInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'POIInfo
    (cl:cons ':name (name msg))
    (cl:cons ':type (type msg))
    (cl:cons ':description (description msg))
    (cl:cons ':position (position msg))
    (cl:cons ':candidate_points (candidate_points msg))
    (cl:cons ':boundary_min (boundary_min msg))
    (cl:cons ':boundary_max (boundary_max msg))
    (cl:cons ':has_boundary (has_boundary msg))
))
