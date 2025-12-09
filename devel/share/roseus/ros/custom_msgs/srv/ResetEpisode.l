;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::ResetEpisode)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'ResetEpisode (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::RESETEPISODE")
  (make-package "CUSTOM_MSGS::RESETEPISODE"))
(unless (find-package "CUSTOM_MSGS::RESETEPISODEREQUEST")
  (make-package "CUSTOM_MSGS::RESETEPISODEREQUEST"))
(unless (find-package "CUSTOM_MSGS::RESETEPISODERESPONSE")
  (make-package "CUSTOM_MSGS::RESETEPISODERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass custom_msgs::ResetEpisodeRequest
  :super ros::object
  :slots ())

(defmethod custom_msgs::ResetEpisodeRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass custom_msgs::ResetEpisodeResponse
  :super ros::object
  :slots (_success _message _start_pose _goal_position ))

(defmethod custom_msgs::ResetEpisodeResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:start_pose __start_pose) (instance geometry_msgs::Pose :init))
    ((:goal_position __goal_position) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _start_pose __start_pose)
   (setq _goal_position __goal_position)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:start_pose
   (&rest __start_pose)
   (if (keywordp (car __start_pose))
       (send* _start_pose __start_pose)
     (progn
       (if __start_pose (setq _start_pose (car __start_pose)))
       _start_pose)))
  (:goal_position
   (&rest __goal_position)
   (if (keywordp (car __goal_position))
       (send* _goal_position __goal_position)
     (progn
       (if __goal_position (setq _goal_position (car __goal_position)))
       _goal_position)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; geometry_msgs/Pose _start_pose
    (send _start_pose :serialization-length)
    ;; geometry_msgs/Point _goal_position
    (send _goal_position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; geometry_msgs/Pose _start_pose
       (send _start_pose :serialize s)
     ;; geometry_msgs/Point _goal_position
       (send _goal_position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _start_pose
     (send _start_pose :deserialize buf ptr-) (incf ptr- (send _start_pose :serialization-length))
   ;; geometry_msgs/Point _goal_position
     (send _goal_position :deserialize buf ptr-) (incf ptr- (send _goal_position :serialization-length))
   ;;
   self)
  )

(defclass custom_msgs::ResetEpisode
  :super ros::object
  :slots ())

(setf (get custom_msgs::ResetEpisode :md5sum-) "e566690f9c9e0029132e05fa58e0f836")
(setf (get custom_msgs::ResetEpisode :datatype-) "custom_msgs/ResetEpisode")
(setf (get custom_msgs::ResetEpisode :request) custom_msgs::ResetEpisodeRequest)
(setf (get custom_msgs::ResetEpisode :response) custom_msgs::ResetEpisodeResponse)

(defmethod custom_msgs::ResetEpisodeRequest
  (:response () (instance custom_msgs::ResetEpisodeResponse :init)))

(setf (get custom_msgs::ResetEpisodeRequest :md5sum-) "e566690f9c9e0029132e05fa58e0f836")
(setf (get custom_msgs::ResetEpisodeRequest :datatype-) "custom_msgs/ResetEpisodeRequest")
(setf (get custom_msgs::ResetEpisodeRequest :definition-)
      "# custom_msgs/srv/ResetEpisode.srv

# --- Request ---
# 请求可以是空的，因为我们只是想触发一个重置
# 可以加一个bool来支持课程学习，但现在先保持简单
# bool use_curriculum

---

# --- Response ---
# Unity找到的安全点将在这里返回
bool success          # 标记重置是否成功
string message        # 附带信息 (例如，失败原因)
geometry_msgs/Pose start_pose
geometry_msgs/Point goal_position


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get custom_msgs::ResetEpisodeResponse :md5sum-) "e566690f9c9e0029132e05fa58e0f836")
(setf (get custom_msgs::ResetEpisodeResponse :datatype-) "custom_msgs/ResetEpisodeResponse")
(setf (get custom_msgs::ResetEpisodeResponse :definition-)
      "# custom_msgs/srv/ResetEpisode.srv

# --- Request ---
# 请求可以是空的，因为我们只是想触发一个重置
# 可以加一个bool来支持课程学习，但现在先保持简单
# bool use_curriculum

---

# --- Response ---
# Unity找到的安全点将在这里返回
bool success          # 标记重置是否成功
string message        # 附带信息 (例如，失败原因)
geometry_msgs/Pose start_pose
geometry_msgs/Point goal_position


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :custom_msgs/ResetEpisode "e566690f9c9e0029132e05fa58e0f836")


