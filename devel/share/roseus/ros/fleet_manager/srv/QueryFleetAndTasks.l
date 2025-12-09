;; Auto-generated. Do not edit!


(when (boundp 'fleet_manager::QueryFleetAndTasks)
  (if (not (find-package "FLEET_MANAGER"))
    (make-package "FLEET_MANAGER"))
  (shadow 'QueryFleetAndTasks (find-package "FLEET_MANAGER")))
(unless (find-package "FLEET_MANAGER::QUERYFLEETANDTASKS")
  (make-package "FLEET_MANAGER::QUERYFLEETANDTASKS"))
(unless (find-package "FLEET_MANAGER::QUERYFLEETANDTASKSREQUEST")
  (make-package "FLEET_MANAGER::QUERYFLEETANDTASKSREQUEST"))
(unless (find-package "FLEET_MANAGER::QUERYFLEETANDTASKSRESPONSE")
  (make-package "FLEET_MANAGER::QUERYFLEETANDTASKSRESPONSE"))

(in-package "ROS")





(defclass fleet_manager::QueryFleetAndTasksRequest
  :super ros::object
  :slots ())

(defmethod fleet_manager::QueryFleetAndTasksRequest
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

(defclass fleet_manager::QueryFleetAndTasksResponse
  :super ros::object
  :slots (_drones _tasks ))

(defmethod fleet_manager::QueryFleetAndTasksResponse
  (:init
   (&key
    ((:drones __drones) ())
    ((:tasks __tasks) ())
    )
   (send-super :init)
   (setq _drones __drones)
   (setq _tasks __tasks)
   self)
  (:drones
   (&rest __drones)
   (if (keywordp (car __drones))
       (send* _drones __drones)
     (progn
       (if __drones (setq _drones (car __drones)))
       _drones)))
  (:tasks
   (&rest __tasks)
   (if (keywordp (car __tasks))
       (send* _tasks __tasks)
     (progn
       (if __tasks (setq _tasks (car __tasks)))
       _tasks)))
  (:serialization-length
   ()
   (+
    ;; fleet_manager/DroneStatus[] _drones
    (apply #'+ (send-all _drones :serialization-length)) 4
    ;; fleet_manager/TaskStatus[] _tasks
    (apply #'+ (send-all _tasks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; fleet_manager/DroneStatus[] _drones
     (write-long (length _drones) s)
     (dolist (elem _drones)
       (send elem :serialize s)
       )
     ;; fleet_manager/TaskStatus[] _tasks
     (write-long (length _tasks) s)
     (dolist (elem _tasks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; fleet_manager/DroneStatus[] _drones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _drones (let (r) (dotimes (i n) (push (instance fleet_manager::DroneStatus :init) r)) r))
     (dolist (elem- _drones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; fleet_manager/TaskStatus[] _tasks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tasks (let (r) (dotimes (i n) (push (instance fleet_manager::TaskStatus :init) r)) r))
     (dolist (elem- _tasks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass fleet_manager::QueryFleetAndTasks
  :super ros::object
  :slots ())

(setf (get fleet_manager::QueryFleetAndTasks :md5sum-) "a9cee634e5450b9a2e85c18da8e84744")
(setf (get fleet_manager::QueryFleetAndTasks :datatype-) "fleet_manager/QueryFleetAndTasks")
(setf (get fleet_manager::QueryFleetAndTasks :request) fleet_manager::QueryFleetAndTasksRequest)
(setf (get fleet_manager::QueryFleetAndTasks :response) fleet_manager::QueryFleetAndTasksResponse)

(defmethod fleet_manager::QueryFleetAndTasksRequest
  (:response () (instance fleet_manager::QueryFleetAndTasksResponse :init)))

(setf (get fleet_manager::QueryFleetAndTasksRequest :md5sum-) "a9cee634e5450b9a2e85c18da8e84744")
(setf (get fleet_manager::QueryFleetAndTasksRequest :datatype-) "fleet_manager/QueryFleetAndTasksRequest")
(setf (get fleet_manager::QueryFleetAndTasksRequest :definition-)
      "# 请求部分为空
---
# 响应部分
fleet_manager/DroneStatus[] drones
fleet_manager/TaskStatus[] tasks


================================================================================
MSG: fleet_manager/DroneStatus
# [修改] 在消息定义中添加常量
# Drone Status Constants
uint8 UNKNOWN=0
uint8 ACTIVE=1
uint8 NEEDS_BACKUP=2
uint8 OFFLINE=3

# Message Fields
string drone_id
uint8 status
int32 last_completed_task_index
geometry_msgs/Pose last_known_pose

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

")

(setf (get fleet_manager::QueryFleetAndTasksResponse :md5sum-) "a9cee634e5450b9a2e85c18da8e84744")
(setf (get fleet_manager::QueryFleetAndTasksResponse :datatype-) "fleet_manager/QueryFleetAndTasksResponse")
(setf (get fleet_manager::QueryFleetAndTasksResponse :definition-)
      "# 请求部分为空
---
# 响应部分
fleet_manager/DroneStatus[] drones
fleet_manager/TaskStatus[] tasks


================================================================================
MSG: fleet_manager/DroneStatus
# [修改] 在消息定义中添加常量
# Drone Status Constants
uint8 UNKNOWN=0
uint8 ACTIVE=1
uint8 NEEDS_BACKUP=2
uint8 OFFLINE=3

# Message Fields
string drone_id
uint8 status
int32 last_completed_task_index
geometry_msgs/Pose last_known_pose

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

")



(provide :fleet_manager/QueryFleetAndTasks "a9cee634e5450b9a2e85c18da8e84744")


