;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::UpdateCurriculum)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'UpdateCurriculum (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::UPDATECURRICULUM")
  (make-package "CUSTOM_MSGS::UPDATECURRICULUM"))
(unless (find-package "CUSTOM_MSGS::UPDATECURRICULUMREQUEST")
  (make-package "CUSTOM_MSGS::UPDATECURRICULUMREQUEST"))
(unless (find-package "CUSTOM_MSGS::UPDATECURRICULUMRESPONSE")
  (make-package "CUSTOM_MSGS::UPDATECURRICULUMRESPONSE"))

(in-package "ROS")





(defclass custom_msgs::UpdateCurriculumRequest
  :super ros::object
  :slots (_spawn_range_x _spawn_range_z _local_target_range ))

(defmethod custom_msgs::UpdateCurriculumRequest
  (:init
   (&key
    ((:spawn_range_x __spawn_range_x) 0.0)
    ((:spawn_range_z __spawn_range_z) 0.0)
    ((:local_target_range __local_target_range) 0.0)
    )
   (send-super :init)
   (setq _spawn_range_x (float __spawn_range_x))
   (setq _spawn_range_z (float __spawn_range_z))
   (setq _local_target_range (float __local_target_range))
   self)
  (:spawn_range_x
   (&optional __spawn_range_x)
   (if __spawn_range_x (setq _spawn_range_x __spawn_range_x)) _spawn_range_x)
  (:spawn_range_z
   (&optional __spawn_range_z)
   (if __spawn_range_z (setq _spawn_range_z __spawn_range_z)) _spawn_range_z)
  (:local_target_range
   (&optional __local_target_range)
   (if __local_target_range (setq _local_target_range __local_target_range)) _local_target_range)
  (:serialization-length
   ()
   (+
    ;; float32 _spawn_range_x
    4
    ;; float32 _spawn_range_z
    4
    ;; float32 _local_target_range
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _spawn_range_x
       (sys::poke _spawn_range_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _spawn_range_z
       (sys::poke _spawn_range_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _local_target_range
       (sys::poke _local_target_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _spawn_range_x
     (setq _spawn_range_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _spawn_range_z
     (setq _spawn_range_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _local_target_range
     (setq _local_target_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass custom_msgs::UpdateCurriculumResponse
  :super ros::object
  :slots (_success ))

(defmethod custom_msgs::UpdateCurriculumResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass custom_msgs::UpdateCurriculum
  :super ros::object
  :slots ())

(setf (get custom_msgs::UpdateCurriculum :md5sum-) "3b2cd8798f0550ceaa4d103980369410")
(setf (get custom_msgs::UpdateCurriculum :datatype-) "custom_msgs/UpdateCurriculum")
(setf (get custom_msgs::UpdateCurriculum :request) custom_msgs::UpdateCurriculumRequest)
(setf (get custom_msgs::UpdateCurriculum :response) custom_msgs::UpdateCurriculumResponse)

(defmethod custom_msgs::UpdateCurriculumRequest
  (:response () (instance custom_msgs::UpdateCurriculumResponse :init)))

(setf (get custom_msgs::UpdateCurriculumRequest :md5sum-) "3b2cd8798f0550ceaa4d103980369410")
(setf (get custom_msgs::UpdateCurriculumRequest :datatype-) "custom_msgs/UpdateCurriculumRequest")
(setf (get custom_msgs::UpdateCurriculumRequest :definition-)
      "float32 spawn_range_x
float32 spawn_range_z
float32 local_target_range
---
bool success

")

(setf (get custom_msgs::UpdateCurriculumResponse :md5sum-) "3b2cd8798f0550ceaa4d103980369410")
(setf (get custom_msgs::UpdateCurriculumResponse :datatype-) "custom_msgs/UpdateCurriculumResponse")
(setf (get custom_msgs::UpdateCurriculumResponse :definition-)
      "float32 spawn_range_x
float32 spawn_range_z
float32 local_target_range
---
bool success

")



(provide :custom_msgs/UpdateCurriculum "3b2cd8798f0550ceaa4d103980369410")


