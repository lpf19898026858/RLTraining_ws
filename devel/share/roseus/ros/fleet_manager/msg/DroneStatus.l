;; Auto-generated. Do not edit!


(when (boundp 'fleet_manager::DroneStatus)
  (if (not (find-package "FLEET_MANAGER"))
    (make-package "FLEET_MANAGER"))
  (shadow 'DroneStatus (find-package "FLEET_MANAGER")))
(unless (find-package "FLEET_MANAGER::DRONESTATUS")
  (make-package "FLEET_MANAGER::DRONESTATUS"))

(in-package "ROS")
;;//! \htmlinclude DroneStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*UNKNOWN*" (find-package "FLEET_MANAGER::DRONESTATUS"))
(shadow '*UNKNOWN* (find-package "FLEET_MANAGER::DRONESTATUS"))
(defconstant fleet_manager::DroneStatus::*UNKNOWN* 0)
(intern "*ACTIVE*" (find-package "FLEET_MANAGER::DRONESTATUS"))
(shadow '*ACTIVE* (find-package "FLEET_MANAGER::DRONESTATUS"))
(defconstant fleet_manager::DroneStatus::*ACTIVE* 1)
(intern "*NEEDS_BACKUP*" (find-package "FLEET_MANAGER::DRONESTATUS"))
(shadow '*NEEDS_BACKUP* (find-package "FLEET_MANAGER::DRONESTATUS"))
(defconstant fleet_manager::DroneStatus::*NEEDS_BACKUP* 2)
(intern "*OFFLINE*" (find-package "FLEET_MANAGER::DRONESTATUS"))
(shadow '*OFFLINE* (find-package "FLEET_MANAGER::DRONESTATUS"))
(defconstant fleet_manager::DroneStatus::*OFFLINE* 3)

(defun fleet_manager::DroneStatus-to-symbol (const)
  (cond
        ((= const 0) 'fleet_manager::DroneStatus::*UNKNOWN*)
        ((= const 1) 'fleet_manager::DroneStatus::*ACTIVE*)
        ((= const 2) 'fleet_manager::DroneStatus::*NEEDS_BACKUP*)
        ((= const 3) 'fleet_manager::DroneStatus::*OFFLINE*)
        (t nil)))

(defclass fleet_manager::DroneStatus
  :super ros::object
  :slots (_drone_id _status _last_completed_task_index _last_known_pose ))

(defmethod fleet_manager::DroneStatus
  (:init
   (&key
    ((:drone_id __drone_id) "")
    ((:status __status) 0)
    ((:last_completed_task_index __last_completed_task_index) 0)
    ((:last_known_pose __last_known_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _drone_id (string __drone_id))
   (setq _status (round __status))
   (setq _last_completed_task_index (round __last_completed_task_index))
   (setq _last_known_pose __last_known_pose)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:last_completed_task_index
   (&optional __last_completed_task_index)
   (if __last_completed_task_index (setq _last_completed_task_index __last_completed_task_index)) _last_completed_task_index)
  (:last_known_pose
   (&rest __last_known_pose)
   (if (keywordp (car __last_known_pose))
       (send* _last_known_pose __last_known_pose)
     (progn
       (if __last_known_pose (setq _last_known_pose (car __last_known_pose)))
       _last_known_pose)))
  (:serialization-length
   ()
   (+
    ;; string _drone_id
    4 (length _drone_id)
    ;; uint8 _status
    1
    ;; int32 _last_completed_task_index
    4
    ;; geometry_msgs/Pose _last_known_pose
    (send _last_known_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _drone_id
       (write-long (length _drone_id) s) (princ _drone_id s)
     ;; uint8 _status
       (write-byte _status s)
     ;; int32 _last_completed_task_index
       (write-long _last_completed_task_index s)
     ;; geometry_msgs/Pose _last_known_pose
       (send _last_known_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _drone_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _drone_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _last_completed_task_index
     (setq _last_completed_task_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _last_known_pose
     (send _last_known_pose :deserialize buf ptr-) (incf ptr- (send _last_known_pose :serialization-length))
   ;;
   self)
  )

(setf (get fleet_manager::DroneStatus :md5sum-) "4399a33682bd42bcbd894637e74a998e")
(setf (get fleet_manager::DroneStatus :datatype-) "fleet_manager/DroneStatus")
(setf (get fleet_manager::DroneStatus :definition-)
      "# [修改] 在消息定义中添加常量
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

")



(provide :fleet_manager/DroneStatus "4399a33682bd42bcbd894637e74a998e")


