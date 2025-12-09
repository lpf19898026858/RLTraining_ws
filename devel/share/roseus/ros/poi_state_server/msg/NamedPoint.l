;; Auto-generated. Do not edit!


(when (boundp 'poi_state_server::NamedPoint)
  (if (not (find-package "POI_STATE_SERVER"))
    (make-package "POI_STATE_SERVER"))
  (shadow 'NamedPoint (find-package "POI_STATE_SERVER")))
(unless (find-package "POI_STATE_SERVER::NAMEDPOINT")
  (make-package "POI_STATE_SERVER::NAMEDPOINT"))

(in-package "ROS")
;;//! \htmlinclude NamedPoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass poi_state_server::NamedPoint
  :super ros::object
  :slots (_name _point ))

(defmethod poi_state_server::NamedPoint
  (:init
   (&key
    ((:name __name) "")
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _point __point)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get poi_state_server::NamedPoint :md5sum-) "679c7c057ebc773242bf1f625c1c9996")
(setf (get poi_state_server::NamedPoint :datatype-) "poi_state_server/NamedPoint")
(setf (get poi_state_server::NamedPoint :definition-)
      "string name
geometry_msgs/Point point


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :poi_state_server/NamedPoint "679c7c057ebc773242bf1f625c1c9996")


