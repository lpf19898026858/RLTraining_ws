;; Auto-generated. Do not edit!


(when (boundp 'vlm_service::CaptureImage)
  (if (not (find-package "VLM_SERVICE"))
    (make-package "VLM_SERVICE"))
  (shadow 'CaptureImage (find-package "VLM_SERVICE")))
(unless (find-package "VLM_SERVICE::CAPTUREIMAGE")
  (make-package "VLM_SERVICE::CAPTUREIMAGE"))
(unless (find-package "VLM_SERVICE::CAPTUREIMAGEREQUEST")
  (make-package "VLM_SERVICE::CAPTUREIMAGEREQUEST"))
(unless (find-package "VLM_SERVICE::CAPTUREIMAGERESPONSE")
  (make-package "VLM_SERVICE::CAPTUREIMAGERESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass vlm_service::CaptureImageRequest
  :super ros::object
  :slots (_directory_path _file_name _image_topic ))

(defmethod vlm_service::CaptureImageRequest
  (:init
   (&key
    ((:directory_path __directory_path) "")
    ((:file_name __file_name) "")
    ((:image_topic __image_topic) "")
    )
   (send-super :init)
   (setq _directory_path (string __directory_path))
   (setq _file_name (string __file_name))
   (setq _image_topic (string __image_topic))
   self)
  (:directory_path
   (&optional __directory_path)
   (if __directory_path (setq _directory_path __directory_path)) _directory_path)
  (:file_name
   (&optional __file_name)
   (if __file_name (setq _file_name __file_name)) _file_name)
  (:image_topic
   (&optional __image_topic)
   (if __image_topic (setq _image_topic __image_topic)) _image_topic)
  (:serialization-length
   ()
   (+
    ;; string _directory_path
    4 (length _directory_path)
    ;; string _file_name
    4 (length _file_name)
    ;; string _image_topic
    4 (length _image_topic)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _directory_path
       (write-long (length _directory_path) s) (princ _directory_path s)
     ;; string _file_name
       (write-long (length _file_name) s) (princ _file_name s)
     ;; string _image_topic
       (write-long (length _image_topic) s) (princ _image_topic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _directory_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _directory_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _image_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vlm_service::CaptureImageResponse
  :super ros::object
  :slots (_success _message _file_path _captured_image ))

(defmethod vlm_service::CaptureImageResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:file_path __file_path) "")
    ((:captured_image __captured_image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _file_path (string __file_path))
   (setq _captured_image __captured_image)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:file_path
   (&optional __file_path)
   (if __file_path (setq _file_path __file_path)) _file_path)
  (:captured_image
   (&rest __captured_image)
   (if (keywordp (car __captured_image))
       (send* _captured_image __captured_image)
     (progn
       (if __captured_image (setq _captured_image (car __captured_image)))
       _captured_image)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; string _file_path
    4 (length _file_path)
    ;; sensor_msgs/Image _captured_image
    (send _captured_image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; string _file_path
       (write-long (length _file_path) s) (princ _file_path s)
     ;; sensor_msgs/Image _captured_image
       (send _captured_image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _file_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _file_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; sensor_msgs/Image _captured_image
     (send _captured_image :deserialize buf ptr-) (incf ptr- (send _captured_image :serialization-length))
   ;;
   self)
  )

(defclass vlm_service::CaptureImage
  :super ros::object
  :slots ())

(setf (get vlm_service::CaptureImage :md5sum-) "e3c78bc34cba5b744044b3a476675c16")
(setf (get vlm_service::CaptureImage :datatype-) "vlm_service/CaptureImage")
(setf (get vlm_service::CaptureImage :request) vlm_service::CaptureImageRequest)
(setf (get vlm_service::CaptureImage :response) vlm_service::CaptureImageResponse)

(defmethod vlm_service::CaptureImageRequest
  (:response () (instance vlm_service::CaptureImageResponse :init)))

(setf (get vlm_service::CaptureImageRequest :md5sum-) "e3c78bc34cba5b744044b3a476675c16")
(setf (get vlm_service::CaptureImageRequest :datatype-) "vlm_service/CaptureImageRequest")
(setf (get vlm_service::CaptureImageRequest :definition-)
      "# Request: 指定保存图像的位置和文件名
string directory_path
string file_name
string image_topic  #指定要捕获的图像话题
---
# Response: 返回操作结果和保存的完整路径
bool success
string message
string file_path # 返回完整路径，方便客户端（如Unity）确认
sensor_msgs/Image captured_image  


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
")

(setf (get vlm_service::CaptureImageResponse :md5sum-) "e3c78bc34cba5b744044b3a476675c16")
(setf (get vlm_service::CaptureImageResponse :datatype-) "vlm_service/CaptureImageResponse")
(setf (get vlm_service::CaptureImageResponse :definition-)
      "# Request: 指定保存图像的位置和文件名
string directory_path
string file_name
string image_topic  #指定要捕获的图像话题
---
# Response: 返回操作结果和保存的完整路径
bool success
string message
string file_path # 返回完整路径，方便客户端（如Unity）确认
sensor_msgs/Image captured_image  


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
")



(provide :vlm_service/CaptureImage "e3c78bc34cba5b744044b3a476675c16")


