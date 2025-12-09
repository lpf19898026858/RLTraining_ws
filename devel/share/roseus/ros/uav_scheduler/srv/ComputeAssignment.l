;; Auto-generated. Do not edit!


(when (boundp 'uav_scheduler::ComputeAssignment)
  (if (not (find-package "UAV_SCHEDULER"))
    (make-package "UAV_SCHEDULER"))
  (shadow 'ComputeAssignment (find-package "UAV_SCHEDULER")))
(unless (find-package "UAV_SCHEDULER::COMPUTEASSIGNMENT")
  (make-package "UAV_SCHEDULER::COMPUTEASSIGNMENT"))
(unless (find-package "UAV_SCHEDULER::COMPUTEASSIGNMENTREQUEST")
  (make-package "UAV_SCHEDULER::COMPUTEASSIGNMENTREQUEST"))
(unless (find-package "UAV_SCHEDULER::COMPUTEASSIGNMENTRESPONSE")
  (make-package "UAV_SCHEDULER::COMPUTEASSIGNMENTRESPONSE"))

(in-package "ROS")





(defclass uav_scheduler::ComputeAssignmentRequest
  :super ros::object
  :slots (_algorithm _drones _pois ))

(defmethod uav_scheduler::ComputeAssignmentRequest
  (:init
   (&key
    ((:algorithm __algorithm) "")
    ((:drones __drones) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:pois __pois) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _algorithm (string __algorithm))
   (setq _drones __drones)
   (setq _pois __pois)
   self)
  (:algorithm
   (&optional __algorithm)
   (if __algorithm (setq _algorithm __algorithm)) _algorithm)
  (:drones
   (&optional __drones)
   (if __drones (setq _drones __drones)) _drones)
  (:pois
   (&optional __pois)
   (if __pois (setq _pois __pois)) _pois)
  (:serialization-length
   ()
   (+
    ;; string _algorithm
    4 (length _algorithm)
    ;; string[] _drones
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _drones)) 4
    ;; string[] _pois
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _pois)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _algorithm
       (write-long (length _algorithm) s) (princ _algorithm s)
     ;; string[] _drones
     (write-long (length _drones) s)
     (dolist (elem _drones)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _pois
     (write-long (length _pois) s)
     (dolist (elem _pois)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _algorithm
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _algorithm (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _drones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _drones (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _drones i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _pois
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pois (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _pois i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass uav_scheduler::ComputeAssignmentResponse
  :super ros::object
  :slots (_assignments _plan_text ))

(defmethod uav_scheduler::ComputeAssignmentResponse
  (:init
   (&key
    ((:assignments __assignments) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:plan_text __plan_text) "")
    )
   (send-super :init)
   (setq _assignments __assignments)
   (setq _plan_text (string __plan_text))
   self)
  (:assignments
   (&optional __assignments)
   (if __assignments (setq _assignments __assignments)) _assignments)
  (:plan_text
   (&optional __plan_text)
   (if __plan_text (setq _plan_text __plan_text)) _plan_text)
  (:serialization-length
   ()
   (+
    ;; string[] _assignments
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _assignments)) 4
    ;; string _plan_text
    4 (length _plan_text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _assignments
     (write-long (length _assignments) s)
     (dolist (elem _assignments)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _plan_text
       (write-long (length _plan_text) s) (princ _plan_text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _assignments
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _assignments (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _assignments i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _plan_text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plan_text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass uav_scheduler::ComputeAssignment
  :super ros::object
  :slots ())

(setf (get uav_scheduler::ComputeAssignment :md5sum-) "97370e26157faf720c13585eba94765b")
(setf (get uav_scheduler::ComputeAssignment :datatype-) "uav_scheduler/ComputeAssignment")
(setf (get uav_scheduler::ComputeAssignment :request) uav_scheduler::ComputeAssignmentRequest)
(setf (get uav_scheduler::ComputeAssignment :response) uav_scheduler::ComputeAssignmentResponse)

(defmethod uav_scheduler::ComputeAssignmentRequest
  (:response () (instance uav_scheduler::ComputeAssignmentResponse :init)))

(setf (get uav_scheduler::ComputeAssignmentRequest :md5sum-) "97370e26157faf720c13585eba94765b")
(setf (get uav_scheduler::ComputeAssignmentRequest :datatype-) "uav_scheduler/ComputeAssignmentRequest")
(setf (get uav_scheduler::ComputeAssignmentRequest :definition-)
      "# 输入
string algorithm              # \"hungarian\", \"auction\", \"genetic\", \"distributed_auction\"
string[] drones
string[] pois
---
# 输出
string[] assignments          # 匹配结果，例如 V_UAV_0->House1
string plan_text              # 任务动作文本


")

(setf (get uav_scheduler::ComputeAssignmentResponse :md5sum-) "97370e26157faf720c13585eba94765b")
(setf (get uav_scheduler::ComputeAssignmentResponse :datatype-) "uav_scheduler/ComputeAssignmentResponse")
(setf (get uav_scheduler::ComputeAssignmentResponse :definition-)
      "# 输入
string algorithm              # \"hungarian\", \"auction\", \"genetic\", \"distributed_auction\"
string[] drones
string[] pois
---
# 输出
string[] assignments          # 匹配结果，例如 V_UAV_0->House1
string plan_text              # 任务动作文本


")



(provide :uav_scheduler/ComputeAssignment "97370e26157faf720c13585eba94765b")


