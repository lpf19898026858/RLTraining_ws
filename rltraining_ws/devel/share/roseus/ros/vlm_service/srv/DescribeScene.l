;; Auto-generated. Do not edit!


(when (boundp 'vlm_service::DescribeScene)
  (if (not (find-package "VLM_SERVICE"))
    (make-package "VLM_SERVICE"))
  (shadow 'DescribeScene (find-package "VLM_SERVICE")))
(unless (find-package "VLM_SERVICE::DESCRIBESCENE")
  (make-package "VLM_SERVICE::DESCRIBESCENE"))
(unless (find-package "VLM_SERVICE::DESCRIBESCENEREQUEST")
  (make-package "VLM_SERVICE::DESCRIBESCENEREQUEST"))
(unless (find-package "VLM_SERVICE::DESCRIBESCENERESPONSE")
  (make-package "VLM_SERVICE::DESCRIBESCENERESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass vlm_service::DescribeSceneRequest
  :super ros::object
  :slots (_prompt _images ))

(defmethod vlm_service::DescribeSceneRequest
  (:init
   (&key
    ((:prompt __prompt) "")
    ((:images __images) ())
    )
   (send-super :init)
   (setq _prompt (string __prompt))
   (setq _images __images)
   self)
  (:prompt
   (&optional __prompt)
   (if __prompt (setq _prompt __prompt)) _prompt)
  (:images
   (&rest __images)
   (if (keywordp (car __images))
       (send* _images __images)
     (progn
       (if __images (setq _images (car __images)))
       _images)))
  (:serialization-length
   ()
   (+
    ;; string _prompt
    4 (length _prompt)
    ;; sensor_msgs/Image[] _images
    (apply #'+ (send-all _images :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _prompt
       (write-long (length _prompt) s) (princ _prompt s)
     ;; sensor_msgs/Image[] _images
     (write-long (length _images) s)
     (dolist (elem _images)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _prompt
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _prompt (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/Image[] _images
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _images (let (r) (dotimes (i n) (push (instance sensor_msgs::Image :init) r)) r))
     (dolist (elem- _images)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass vlm_service::DescribeSceneResponse
  :super ros::object
  :slots (_success _description ))

(defmethod vlm_service::DescribeSceneResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _description (string __description))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vlm_service::DescribeScene
  :super ros::object
  :slots ())

(setf (get vlm_service::DescribeScene :md5sum-) "edba9cc6ddda26cb7fba80817828a2a9")
(setf (get vlm_service::DescribeScene :datatype-) "vlm_service/DescribeScene")
(setf (get vlm_service::DescribeScene :request) vlm_service::DescribeSceneRequest)
(setf (get vlm_service::DescribeScene :response) vlm_service::DescribeSceneResponse)

(defmethod vlm_service::DescribeSceneRequest
  (:response () (instance vlm_service::DescribeSceneResponse :init)))

(setf (get vlm_service::DescribeSceneRequest :md5sum-) "edba9cc6ddda26cb7fba80817828a2a9")
(setf (get vlm_service::DescribeSceneRequest :datatype-) "vlm_service/DescribeSceneRequest")
(setf (get vlm_service::DescribeSceneRequest :definition-)
      "# Request part (can be empty if we always use the latest image)
string prompt  # 允许调用者指定一个问题，例如 \"Is the window open?\"
sensor_msgs/Image[] images  # <-- 关键修改：从单个隐式图像变为一个显式图像数组

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
# Response part
bool success   # 指示VLM调用是否成功
string description # VLM返回的文本描述

")

(setf (get vlm_service::DescribeSceneResponse :md5sum-) "edba9cc6ddda26cb7fba80817828a2a9")
(setf (get vlm_service::DescribeSceneResponse :datatype-) "vlm_service/DescribeSceneResponse")
(setf (get vlm_service::DescribeSceneResponse :definition-)
      "# Request part (can be empty if we always use the latest image)
string prompt  # 允许调用者指定一个问题，例如 \"Is the window open?\"
sensor_msgs/Image[] images  # <-- 关键修改：从单个隐式图像变为一个显式图像数组

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
---
# Response part
bool success   # 指示VLM调用是否成功
string description # VLM返回的文本描述

")



(provide :vlm_service/DescribeScene "edba9cc6ddda26cb7fba80817828a2a9")


