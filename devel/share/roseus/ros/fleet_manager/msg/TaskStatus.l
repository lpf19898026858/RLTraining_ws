;; Auto-generated. Do not edit!


(when (boundp 'fleet_manager::TaskStatus)
  (if (not (find-package "FLEET_MANAGER"))
    (make-package "FLEET_MANAGER"))
  (shadow 'TaskStatus (find-package "FLEET_MANAGER")))
(unless (find-package "FLEET_MANAGER::TASKSTATUS")
  (make-package "FLEET_MANAGER::TASKSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TaskStatus.msg.html
(if (not (find-package "NLP_DRONE_CONTROL"))
  (ros::roseus-add-msgs "nlp_drone_control"))


(intern "*PENDING*" (find-package "FLEET_MANAGER::TASKSTATUS"))
(shadow '*PENDING* (find-package "FLEET_MANAGER::TASKSTATUS"))
(defconstant fleet_manager::TaskStatus::*PENDING* 0)
(intern "*IN_PROGRESS*" (find-package "FLEET_MANAGER::TASKSTATUS"))
(shadow '*IN_PROGRESS* (find-package "FLEET_MANAGER::TASKSTATUS"))
(defconstant fleet_manager::TaskStatus::*IN_PROGRESS* 1)
(intern "*COMPLETED*" (find-package "FLEET_MANAGER::TASKSTATUS"))
(shadow '*COMPLETED* (find-package "FLEET_MANAGER::TASKSTATUS"))
(defconstant fleet_manager::TaskStatus::*COMPLETED* 2)
(intern "*NEEDS_BACKUP*" (find-package "FLEET_MANAGER::TASKSTATUS"))
(shadow '*NEEDS_BACKUP* (find-package "FLEET_MANAGER::TASKSTATUS"))
(defconstant fleet_manager::TaskStatus::*NEEDS_BACKUP* 3)

(defun fleet_manager::TaskStatus-to-symbol (const)
  (cond
        ((= const 0) 'fleet_manager::TaskStatus::*PENDING*)
        ((= const 1) 'fleet_manager::TaskStatus::*IN_PROGRESS*)
        ((= const 2) 'fleet_manager::TaskStatus::*COMPLETED*)
        ((= const 3) 'fleet_manager::TaskStatus::*NEEDS_BACKUP*)
        (t nil)))

(defclass fleet_manager::TaskStatus
  :super ros::object
  :slots (_task_id _action_details _status _assigned_drone_id ))

(defmethod fleet_manager::TaskStatus
  (:init
   (&key
    ((:task_id __task_id) 0)
    ((:action_details __action_details) (instance nlp_drone_control::Action :init))
    ((:status __status) 0)
    ((:assigned_drone_id __assigned_drone_id) "")
    )
   (send-super :init)
   (setq _task_id (round __task_id))
   (setq _action_details __action_details)
   (setq _status (round __status))
   (setq _assigned_drone_id (string __assigned_drone_id))
   self)
  (:task_id
   (&optional __task_id)
   (if __task_id (setq _task_id __task_id)) _task_id)
  (:action_details
   (&rest __action_details)
   (if (keywordp (car __action_details))
       (send* _action_details __action_details)
     (progn
       (if __action_details (setq _action_details (car __action_details)))
       _action_details)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:assigned_drone_id
   (&optional __assigned_drone_id)
   (if __assigned_drone_id (setq _assigned_drone_id __assigned_drone_id)) _assigned_drone_id)
  (:serialization-length
   ()
   (+
    ;; int32 _task_id
    4
    ;; nlp_drone_control/Action _action_details
    (send _action_details :serialization-length)
    ;; uint8 _status
    1
    ;; string _assigned_drone_id
    4 (length _assigned_drone_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _task_id
       (write-long _task_id s)
     ;; nlp_drone_control/Action _action_details
       (send _action_details :serialize s)
     ;; uint8 _status
       (write-byte _status s)
     ;; string _assigned_drone_id
       (write-long (length _assigned_drone_id) s) (princ _assigned_drone_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _task_id
     (setq _task_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; nlp_drone_control/Action _action_details
     (send _action_details :deserialize buf ptr-) (incf ptr- (send _action_details :serialization-length))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _assigned_drone_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _assigned_drone_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fleet_manager::TaskStatus :md5sum-) "1a83d6845a55cd58a28f4d61df9680e1")
(setf (get fleet_manager::TaskStatus :datatype-) "fleet_manager/TaskStatus")
(setf (get fleet_manager::TaskStatus :definition-)
      "# [修改] 在消息定义中添加常量
# Task Status Constants
uint8 PENDING=0
uint8 IN_PROGRESS=1
uint8 COMPLETED=2
uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码

# Message Fields
int32 task_id
nlp_drone_control/Action action_details
uint8 status
string assigned_drone_id

================================================================================
MSG: nlp_drone_control/Action
# Represents a single tool call produced by the LLM
string function_name       # e.g. \"takeoff\"
string arguments_json      # raw JSON string of arguments


")



(provide :fleet_manager/TaskStatus "1a83d6845a55cd58a28f4d61df9680e1")


