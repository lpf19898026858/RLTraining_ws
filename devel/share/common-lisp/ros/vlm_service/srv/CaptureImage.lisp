; Auto-generated. Do not edit!


(cl:in-package vlm_service-srv)


;//! \htmlinclude CaptureImage-request.msg.html

(cl:defclass <CaptureImage-request> (roslisp-msg-protocol:ros-message)
  ((directory_path
    :reader directory_path
    :initarg :directory_path
    :type cl:string
    :initform "")
   (file_name
    :reader file_name
    :initarg :file_name
    :type cl:string
    :initform "")
   (image_topic
    :reader image_topic
    :initarg :image_topic
    :type cl:string
    :initform ""))
)

(cl:defclass CaptureImage-request (<CaptureImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vlm_service-srv:<CaptureImage-request> is deprecated: use vlm_service-srv:CaptureImage-request instead.")))

(cl:ensure-generic-function 'directory_path-val :lambda-list '(m))
(cl:defmethod directory_path-val ((m <CaptureImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:directory_path-val is deprecated.  Use vlm_service-srv:directory_path instead.")
  (directory_path m))

(cl:ensure-generic-function 'file_name-val :lambda-list '(m))
(cl:defmethod file_name-val ((m <CaptureImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:file_name-val is deprecated.  Use vlm_service-srv:file_name instead.")
  (file_name m))

(cl:ensure-generic-function 'image_topic-val :lambda-list '(m))
(cl:defmethod image_topic-val ((m <CaptureImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:image_topic-val is deprecated.  Use vlm_service-srv:image_topic instead.")
  (image_topic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-request>) ostream)
  "Serializes a message object of type '<CaptureImage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'directory_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'directory_path))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'image_topic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'image_topic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-request>) istream)
  "Deserializes a message object of type '<CaptureImage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'directory_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'directory_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_topic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'image_topic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-request>)))
  "Returns string type for a service object of type '<CaptureImage-request>"
  "vlm_service/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-request)))
  "Returns string type for a service object of type 'CaptureImage-request"
  "vlm_service/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-request>)))
  "Returns md5sum for a message object of type '<CaptureImage-request>"
  "e3c78bc34cba5b744044b3a476675c16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-request)))
  "Returns md5sum for a message object of type 'CaptureImage-request"
  "e3c78bc34cba5b744044b3a476675c16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-request>)))
  "Returns full string definition for message of type '<CaptureImage-request>"
  (cl:format cl:nil "# Request: 指定保存图像的位置和文件名~%string directory_path~%string file_name~%string image_topic  #指定要捕获的图像话题~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-request)))
  "Returns full string definition for message of type 'CaptureImage-request"
  (cl:format cl:nil "# Request: 指定保存图像的位置和文件名~%string directory_path~%string file_name~%string image_topic  #指定要捕获的图像话题~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'directory_path))
     4 (cl:length (cl:slot-value msg 'file_name))
     4 (cl:length (cl:slot-value msg 'image_topic))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-request
    (cl:cons ':directory_path (directory_path msg))
    (cl:cons ':file_name (file_name msg))
    (cl:cons ':image_topic (image_topic msg))
))
;//! \htmlinclude CaptureImage-response.msg.html

(cl:defclass <CaptureImage-response> (roslisp-msg-protocol:ros-message)
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
   (file_path
    :reader file_path
    :initarg :file_path
    :type cl:string
    :initform "")
   (captured_image
    :reader captured_image
    :initarg :captured_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass CaptureImage-response (<CaptureImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vlm_service-srv:<CaptureImage-response> is deprecated: use vlm_service-srv:CaptureImage-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:success-val is deprecated.  Use vlm_service-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:message-val is deprecated.  Use vlm_service-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'file_path-val :lambda-list '(m))
(cl:defmethod file_path-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:file_path-val is deprecated.  Use vlm_service-srv:file_path instead.")
  (file_path m))

(cl:ensure-generic-function 'captured_image-val :lambda-list '(m))
(cl:defmethod captured_image-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:captured_image-val is deprecated.  Use vlm_service-srv:captured_image instead.")
  (captured_image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-response>) ostream)
  "Serializes a message object of type '<CaptureImage-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file_path))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'captured_image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-response>) istream)
  "Deserializes a message object of type '<CaptureImage-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'captured_image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-response>)))
  "Returns string type for a service object of type '<CaptureImage-response>"
  "vlm_service/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-response)))
  "Returns string type for a service object of type 'CaptureImage-response"
  "vlm_service/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-response>)))
  "Returns md5sum for a message object of type '<CaptureImage-response>"
  "e3c78bc34cba5b744044b3a476675c16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-response)))
  "Returns md5sum for a message object of type 'CaptureImage-response"
  "e3c78bc34cba5b744044b3a476675c16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-response>)))
  "Returns full string definition for message of type '<CaptureImage-response>"
  (cl:format cl:nil "# Response: 返回操作结果和保存的完整路径~%bool success~%string message~%string file_path # 返回完整路径，方便客户端（如Unity）确认~%sensor_msgs/Image captured_image  ~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-response)))
  "Returns full string definition for message of type 'CaptureImage-response"
  (cl:format cl:nil "# Response: 返回操作结果和保存的完整路径~%bool success~%string message~%string file_path # 返回完整路径，方便客户端（如Unity）确认~%sensor_msgs/Image captured_image  ~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4 (cl:length (cl:slot-value msg 'file_path))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'captured_image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':file_path (file_path msg))
    (cl:cons ':captured_image (captured_image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage)))
  "Returns string type for a service object of type '<CaptureImage>"
  "vlm_service/CaptureImage")