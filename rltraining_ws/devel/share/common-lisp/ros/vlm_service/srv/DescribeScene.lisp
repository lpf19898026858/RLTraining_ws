; Auto-generated. Do not edit!


(cl:in-package vlm_service-srv)


;//! \htmlinclude DescribeScene-request.msg.html

(cl:defclass <DescribeScene-request> (roslisp-msg-protocol:ros-message)
  ((prompt
    :reader prompt
    :initarg :prompt
    :type cl:string
    :initform "")
   (images
    :reader images
    :initarg :images
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image))))
)

(cl:defclass DescribeScene-request (<DescribeScene-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DescribeScene-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DescribeScene-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vlm_service-srv:<DescribeScene-request> is deprecated: use vlm_service-srv:DescribeScene-request instead.")))

(cl:ensure-generic-function 'prompt-val :lambda-list '(m))
(cl:defmethod prompt-val ((m <DescribeScene-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:prompt-val is deprecated.  Use vlm_service-srv:prompt instead.")
  (prompt m))

(cl:ensure-generic-function 'images-val :lambda-list '(m))
(cl:defmethod images-val ((m <DescribeScene-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:images-val is deprecated.  Use vlm_service-srv:images instead.")
  (images m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DescribeScene-request>) ostream)
  "Serializes a message object of type '<DescribeScene-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'prompt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'prompt))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'images))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'images))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DescribeScene-request>) istream)
  "Deserializes a message object of type '<DescribeScene-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'prompt) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'prompt) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'images) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'images)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DescribeScene-request>)))
  "Returns string type for a service object of type '<DescribeScene-request>"
  "vlm_service/DescribeSceneRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DescribeScene-request)))
  "Returns string type for a service object of type 'DescribeScene-request"
  "vlm_service/DescribeSceneRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DescribeScene-request>)))
  "Returns md5sum for a message object of type '<DescribeScene-request>"
  "edba9cc6ddda26cb7fba80817828a2a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DescribeScene-request)))
  "Returns md5sum for a message object of type 'DescribeScene-request"
  "edba9cc6ddda26cb7fba80817828a2a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DescribeScene-request>)))
  "Returns full string definition for message of type '<DescribeScene-request>"
  (cl:format cl:nil "# Request part (can be empty if we always use the latest image)~%string prompt  # 允许调用者指定一个问题，例如 \"Is the window open?\"~%sensor_msgs/Image[] images  # <-- 关键修改：从单个隐式图像变为一个显式图像数组~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DescribeScene-request)))
  "Returns full string definition for message of type 'DescribeScene-request"
  (cl:format cl:nil "# Request part (can be empty if we always use the latest image)~%string prompt  # 允许调用者指定一个问题，例如 \"Is the window open?\"~%sensor_msgs/Image[] images  # <-- 关键修改：从单个隐式图像变为一个显式图像数组~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DescribeScene-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'prompt))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'images) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DescribeScene-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DescribeScene-request
    (cl:cons ':prompt (prompt msg))
    (cl:cons ':images (images msg))
))
;//! \htmlinclude DescribeScene-response.msg.html

(cl:defclass <DescribeScene-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass DescribeScene-response (<DescribeScene-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DescribeScene-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DescribeScene-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vlm_service-srv:<DescribeScene-response> is deprecated: use vlm_service-srv:DescribeScene-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DescribeScene-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:success-val is deprecated.  Use vlm_service-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <DescribeScene-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vlm_service-srv:description-val is deprecated.  Use vlm_service-srv:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DescribeScene-response>) ostream)
  "Serializes a message object of type '<DescribeScene-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DescribeScene-response>) istream)
  "Deserializes a message object of type '<DescribeScene-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DescribeScene-response>)))
  "Returns string type for a service object of type '<DescribeScene-response>"
  "vlm_service/DescribeSceneResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DescribeScene-response)))
  "Returns string type for a service object of type 'DescribeScene-response"
  "vlm_service/DescribeSceneResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DescribeScene-response>)))
  "Returns md5sum for a message object of type '<DescribeScene-response>"
  "edba9cc6ddda26cb7fba80817828a2a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DescribeScene-response)))
  "Returns md5sum for a message object of type 'DescribeScene-response"
  "edba9cc6ddda26cb7fba80817828a2a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DescribeScene-response>)))
  "Returns full string definition for message of type '<DescribeScene-response>"
  (cl:format cl:nil "# Response part~%bool success   # 指示VLM调用是否成功~%string description # VLM返回的文本描述~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DescribeScene-response)))
  "Returns full string definition for message of type 'DescribeScene-response"
  (cl:format cl:nil "# Response part~%bool success   # 指示VLM调用是否成功~%string description # VLM返回的文本描述~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DescribeScene-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DescribeScene-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DescribeScene-response
    (cl:cons ':success (success msg))
    (cl:cons ':description (description msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DescribeScene)))
  'DescribeScene-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DescribeScene)))
  'DescribeScene-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DescribeScene)))
  "Returns string type for a service object of type '<DescribeScene>"
  "vlm_service/DescribeScene")