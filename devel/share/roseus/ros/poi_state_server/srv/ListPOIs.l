;; Auto-generated. Do not edit!


(when (boundp 'poi_state_server::ListPOIs)
  (if (not (find-package "POI_STATE_SERVER"))
    (make-package "POI_STATE_SERVER"))
  (shadow 'ListPOIs (find-package "POI_STATE_SERVER")))
(unless (find-package "POI_STATE_SERVER::LISTPOIS")
  (make-package "POI_STATE_SERVER::LISTPOIS"))
(unless (find-package "POI_STATE_SERVER::LISTPOISREQUEST")
  (make-package "POI_STATE_SERVER::LISTPOISREQUEST"))
(unless (find-package "POI_STATE_SERVER::LISTPOISRESPONSE")
  (make-package "POI_STATE_SERVER::LISTPOISRESPONSE"))

(in-package "ROS")





(defclass poi_state_server::ListPOIsRequest
  :super ros::object
  :slots ())

(defmethod poi_state_server::ListPOIsRequest
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

(defclass poi_state_server::ListPOIsResponse
  :super ros::object
  :slots (_success _message _pois ))

(defmethod poi_state_server::ListPOIsResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:pois __pois) ())
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _pois __pois)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:pois
   (&rest __pois)
   (if (keywordp (car __pois))
       (send* _pois __pois)
     (progn
       (if __pois (setq _pois (car __pois)))
       _pois)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; poi_state_server/POIInfo[] _pois
    (apply #'+ (send-all _pois :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; poi_state_server/POIInfo[] _pois
     (write-long (length _pois) s)
     (dolist (elem _pois)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; poi_state_server/POIInfo[] _pois
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pois (let (r) (dotimes (i n) (push (instance poi_state_server::POIInfo :init) r)) r))
     (dolist (elem- _pois)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass poi_state_server::ListPOIs
  :super ros::object
  :slots ())

(setf (get poi_state_server::ListPOIs :md5sum-) "5729bcda884e859dbb5f9dba61bed21b")
(setf (get poi_state_server::ListPOIs :datatype-) "poi_state_server/ListPOIs")
(setf (get poi_state_server::ListPOIs :request) poi_state_server::ListPOIsRequest)
(setf (get poi_state_server::ListPOIs :response) poi_state_server::ListPOIsResponse)

(defmethod poi_state_server::ListPOIsRequest
  (:response () (instance poi_state_server::ListPOIsResponse :init)))

(setf (get poi_state_server::ListPOIsRequest :md5sum-) "5729bcda884e859dbb5f9dba61bed21b")
(setf (get poi_state_server::ListPOIsRequest :datatype-) "poi_state_server/ListPOIsRequest")
(setf (get poi_state_server::ListPOIsRequest :definition-)
      "---
bool success
string message
poi_state_server/POIInfo[] pois



================================================================================
MSG: poi_state_server/POIInfo
# POIInfo.msg
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

(setf (get poi_state_server::ListPOIsResponse :md5sum-) "5729bcda884e859dbb5f9dba61bed21b")
(setf (get poi_state_server::ListPOIsResponse :datatype-) "poi_state_server/ListPOIsResponse")
(setf (get poi_state_server::ListPOIsResponse :definition-)
      "---
bool success
string message
poi_state_server/POIInfo[] pois



================================================================================
MSG: poi_state_server/POIInfo
# POIInfo.msg
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



(provide :poi_state_server/ListPOIs "5729bcda884e859dbb5f9dba61bed21b")


