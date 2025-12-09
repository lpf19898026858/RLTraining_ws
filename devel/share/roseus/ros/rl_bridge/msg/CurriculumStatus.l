;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::CurriculumStatus)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'CurriculumStatus (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::CURRICULUMSTATUS")
  (make-package "RL_BRIDGE::CURRICULUMSTATUS"))

(in-package "ROS")
;;//! \htmlinclude CurriculumStatus.msg.html


(defclass rl_bridge::CurriculumStatus
  :super ros::object
  :slots (_lesson_id _lesson_name _changed ))

(defmethod rl_bridge::CurriculumStatus
  (:init
   (&key
    ((:lesson_id __lesson_id) 0)
    ((:lesson_name __lesson_name) "")
    ((:changed __changed) nil)
    )
   (send-super :init)
   (setq _lesson_id (round __lesson_id))
   (setq _lesson_name (string __lesson_name))
   (setq _changed __changed)
   self)
  (:lesson_id
   (&optional __lesson_id)
   (if __lesson_id (setq _lesson_id __lesson_id)) _lesson_id)
  (:lesson_name
   (&optional __lesson_name)
   (if __lesson_name (setq _lesson_name __lesson_name)) _lesson_name)
  (:changed
   (&optional (__changed :null))
   (if (not (eq __changed :null)) (setq _changed __changed)) _changed)
  (:serialization-length
   ()
   (+
    ;; int32 _lesson_id
    4
    ;; string _lesson_name
    4 (length _lesson_name)
    ;; bool _changed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _lesson_id
       (write-long _lesson_id s)
     ;; string _lesson_name
       (write-long (length _lesson_name) s) (princ _lesson_name s)
     ;; bool _changed
       (if _changed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _lesson_id
     (setq _lesson_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _lesson_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lesson_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _changed
     (setq _changed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rl_bridge::CurriculumStatus :md5sum-) "eed77d69c41fbfca0645d6c4f9105016")
(setf (get rl_bridge::CurriculumStatus :datatype-) "rl_bridge/CurriculumStatus")
(setf (get rl_bridge::CurriculumStatus :definition-)
      "
int32 lesson_id
string lesson_name
bool changed



")



(provide :rl_bridge/CurriculumStatus "eed77d69c41fbfca0645d6c4f9105016")


