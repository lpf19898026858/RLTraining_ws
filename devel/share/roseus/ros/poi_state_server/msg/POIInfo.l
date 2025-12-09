;; Auto-generated. Do not edit!


(when (boundp 'poi_state_server::POIInfo)
  (if (not (find-package "POI_STATE_SERVER"))
    (make-package "POI_STATE_SERVER"))
  (shadow 'POIInfo (find-package "POI_STATE_SERVER")))
(unless (find-package "POI_STATE_SERVER::POIINFO")
  (make-package "POI_STATE_SERVER::POIINFO"))

(in-package "ROS")
;;//! \htmlinclude POIInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass poi_state_server::POIInfo
  :super ros::object
  :slots (_name _type _description _position _candidate_points _boundary_min _boundary_max _has_boundary ))

(defmethod poi_state_server::POIInfo
  (:init
   (&key
    ((:name __name) "")
    ((:type __type) "")
    ((:description __description) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:candidate_points __candidate_points) ())
    ((:boundary_min __boundary_min) (instance geometry_msgs::Point :init))
    ((:boundary_max __boundary_max) (instance geometry_msgs::Point :init))
    ((:has_boundary __has_boundary) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _type (string __type))
   (setq _description (string __description))
   (setq _position __position)
   (setq _candidate_points __candidate_points)
   (setq _boundary_min __boundary_min)
   (setq _boundary_max __boundary_max)
   (setq _has_boundary __has_boundary)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:candidate_points
   (&rest __candidate_points)
   (if (keywordp (car __candidate_points))
       (send* _candidate_points __candidate_points)
     (progn
       (if __candidate_points (setq _candidate_points (car __candidate_points)))
       _candidate_points)))
  (:boundary_min
   (&rest __boundary_min)
   (if (keywordp (car __boundary_min))
       (send* _boundary_min __boundary_min)
     (progn
       (if __boundary_min (setq _boundary_min (car __boundary_min)))
       _boundary_min)))
  (:boundary_max
   (&rest __boundary_max)
   (if (keywordp (car __boundary_max))
       (send* _boundary_max __boundary_max)
     (progn
       (if __boundary_max (setq _boundary_max (car __boundary_max)))
       _boundary_max)))
  (:has_boundary
   (&optional (__has_boundary :null))
   (if (not (eq __has_boundary :null)) (setq _has_boundary __has_boundary)) _has_boundary)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _type
    4 (length _type)
    ;; string _description
    4 (length _description)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; poi_state_server/NamedPoint[] _candidate_points
    (apply #'+ (send-all _candidate_points :serialization-length)) 4
    ;; geometry_msgs/Point _boundary_min
    (send _boundary_min :serialization-length)
    ;; geometry_msgs/Point _boundary_max
    (send _boundary_max :serialization-length)
    ;; bool _has_boundary
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; poi_state_server/NamedPoint[] _candidate_points
     (write-long (length _candidate_points) s)
     (dolist (elem _candidate_points)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point _boundary_min
       (send _boundary_min :serialize s)
     ;; geometry_msgs/Point _boundary_max
       (send _boundary_max :serialize s)
     ;; bool _has_boundary
       (if _has_boundary (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; poi_state_server/NamedPoint[] _candidate_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _candidate_points (let (r) (dotimes (i n) (push (instance poi_state_server::NamedPoint :init) r)) r))
     (dolist (elem- _candidate_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point _boundary_min
     (send _boundary_min :deserialize buf ptr-) (incf ptr- (send _boundary_min :serialization-length))
   ;; geometry_msgs/Point _boundary_max
     (send _boundary_max :deserialize buf ptr-) (incf ptr- (send _boundary_max :serialization-length))
   ;; bool _has_boundary
     (setq _has_boundary (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get poi_state_server::POIInfo :md5sum-) "4b1a999c05b4e892b655e03546fef00e")
(setf (get poi_state_server::POIInfo :datatype-) "poi_state_server/POIInfo")
(setf (get poi_state_server::POIInfo :definition-)
      "# POIInfo.msg
string name
string type
string description
geometry_msgs/Point position
poi_state_server/NamedPoint[] candidate_points
geometry_msgs/Point boundary_min
geometry_msgs/Point boundary_max
bool has_boundary


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: poi_state_server/NamedPoint
string name
geometry_msgs/Point point


")



(provide :poi_state_server/POIInfo "4b1a999c05b4e892b655e03546fef00e")


