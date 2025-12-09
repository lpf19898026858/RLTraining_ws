;; Auto-generated. Do not edit!


(when (boundp 'fleet_manager::InjectTasks)
  (if (not (find-package "FLEET_MANAGER"))
    (make-package "FLEET_MANAGER"))
  (shadow 'InjectTasks (find-package "FLEET_MANAGER")))
(unless (find-package "FLEET_MANAGER::INJECTTASKS")
  (make-package "FLEET_MANAGER::INJECTTASKS"))
(unless (find-package "FLEET_MANAGER::INJECTTASKSREQUEST")
  (make-package "FLEET_MANAGER::INJECTTASKSREQUEST"))
(unless (find-package "FLEET_MANAGER::INJECTTASKSRESPONSE")
  (make-package "FLEET_MANAGER::INJECTTASKSRESPONSE"))

(in-package "ROS")





(defclass fleet_manager::InjectTasksRequest
  :super ros::object
  :slots (_tasks _assign_to_drone_id ))

(defmethod fleet_manager::InjectTasksRequest
  (:init
   (&key
    ((:tasks __tasks) ())
    ((:assign_to_drone_id __assign_to_drone_id) "")
    )
   (send-super :init)
   (setq _tasks __tasks)
   (setq _assign_to_drone_id (string __assign_to_drone_id))
   self)
  (:tasks
   (&rest __tasks)
   (if (keywordp (car __tasks))
       (send* _tasks __tasks)
     (progn
       (if __tasks (setq _tasks (car __tasks)))
       _tasks)))
  (:assign_to_drone_id
   (&optional __assign_to_drone_id)
   (if __assign_to_drone_id (setq _assign_to_drone_id __assign_to_drone_id)) _assign_to_drone_id)
  (:serialization-length
   ()
   (+
    ;; fleet_manager/TaskStatus[] _tasks
    (apply #'+ (send-all _tasks :serialization-length)) 4
    ;; string _assign_to_drone_id
    4 (length _assign_to_drone_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; fleet_manager/TaskStatus[] _tasks
     (write-long (length _tasks) s)
     (dolist (elem _tasks)
       (send elem :serialize s)
       )
     ;; string _assign_to_drone_id
       (write-long (length _assign_to_drone_id) s) (princ _assign_to_drone_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; fleet_manager/TaskStatus[] _tasks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tasks (let (r) (dotimes (i n) (push (instance fleet_manager::TaskStatus :init) r)) r))
     (dolist (elem- _tasks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _assign_to_drone_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _assign_to_drone_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass fleet_manager::InjectTasksResponse
  :super ros::object
  :slots (_success ))

(defmethod fleet_manager::InjectTasksResponse
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

(defclass fleet_manager::InjectTasks
  :super ros::object
  :slots ())

(setf (get fleet_manager::InjectTasks :md5sum-) "72ac7b4be33eb67b134d27e55502c210")
(setf (get fleet_manager::InjectTasks :datatype-) "fleet_manager/InjectTasks")
(setf (get fleet_manager::InjectTasks :request) fleet_manager::InjectTasksRequest)
(setf (get fleet_manager::InjectTasks :response) fleet_manager::InjectTasksResponse)

(defmethod fleet_manager::InjectTasksRequest
  (:response () (instance fleet_manager::InjectTasksResponse :init)))

(setf (get fleet_manager::InjectTasksRequest :md5sum-) "72ac7b4be33eb67b134d27e55502c210")
(setf (get fleet_manager::InjectTasksRequest :datatype-) "fleet_manager/InjectTasksRequest")
(setf (get fleet_manager::InjectTasksRequest :definition-)
      "fleet_manager/TaskStatus[] tasks
string assign_to_drone_id

================================================================================
MSG: fleet_manager/TaskStatus
# [修改] 在消息定义中添加常量
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

---
bool success

")

(setf (get fleet_manager::InjectTasksResponse :md5sum-) "72ac7b4be33eb67b134d27e55502c210")
(setf (get fleet_manager::InjectTasksResponse :datatype-) "fleet_manager/InjectTasksResponse")
(setf (get fleet_manager::InjectTasksResponse :definition-)
      "fleet_manager/TaskStatus[] tasks
string assign_to_drone_id

================================================================================
MSG: fleet_manager/TaskStatus
# [修改] 在消息定义中添加常量
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

---
bool success

")



(provide :fleet_manager/InjectTasks "72ac7b4be33eb67b134d27e55502c210")


