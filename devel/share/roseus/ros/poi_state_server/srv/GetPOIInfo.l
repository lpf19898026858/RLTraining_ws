;; Auto-generated. Do not edit!


(when (boundp 'poi_state_server::GetPOIInfo)
  (if (not (find-package "POI_STATE_SERVER"))
    (make-package "POI_STATE_SERVER"))
  (shadow 'GetPOIInfo (find-package "POI_STATE_SERVER")))
(unless (find-package "POI_STATE_SERVER::GETPOIINFO")
  (make-package "POI_STATE_SERVER::GETPOIINFO"))
(unless (find-package "POI_STATE_SERVER::GETPOIINFOREQUEST")
  (make-package "POI_STATE_SERVER::GETPOIINFOREQUEST"))
(unless (find-package "POI_STATE_SERVER::GETPOIINFORESPONSE")
  (make-package "POI_STATE_SERVER::GETPOIINFORESPONSE"))

(in-package "ROS")





(defclass poi_state_server::GetPOIInfoRequest
  :super ros::object
  :slots (_name ))

(defmethod poi_state_server::GetPOIInfoRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass poi_state_server::GetPOIInfoResponse
  :super ros::object
  :slots (_success _message _info ))

(defmethod poi_state_server::GetPOIInfoResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:info __info) (instance poi_state_server::POIInfo :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _info __info)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:info
   (&rest __info)
   (if (keywordp (car __info))
       (send* _info __info)
     (progn
       (if __info (setq _info (car __info)))
       _info)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; poi_state_server/POIInfo _info
    (send _info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; poi_state_server/POIInfo _info
       (send _info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; poi_state_server/POIInfo _info
     (send _info :deserialize buf ptr-) (incf ptr- (send _info :serialization-length))
   ;;
   self)
  )

(defclass poi_state_server::GetPOIInfo
  :super ros::object
  :slots ())

(setf (get poi_state_server::GetPOIInfo :md5sum-) "c65c76ed817470389da7b2cfd1cdfbea")
(setf (get poi_state_server::GetPOIInfo :datatype-) "poi_state_server/GetPOIInfo")
(setf (get poi_state_server::GetPOIInfo :request) poi_state_server::GetPOIInfoRequest)
(setf (get poi_state_server::GetPOIInfo :response) poi_state_server::GetPOIInfoResponse)

(defmethod poi_state_server::GetPOIInfoRequest
  (:response () (instance poi_state_server::GetPOIInfoResponse :init)))

(setf (get poi_state_server::GetPOIInfoRequest :md5sum-) "c65c76ed817470389da7b2cfd1cdfbea")
(setf (get poi_state_server::GetPOIInfoRequest :datatype-) "poi_state_server/GetPOIInfoRequest")
(setf (get poi_state_server::GetPOIInfoRequest :definition-)
      "string name
---
bool success
string message
poi_state_server/POIInfo info



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

(setf (get poi_state_server::GetPOIInfoResponse :md5sum-) "c65c76ed817470389da7b2cfd1cdfbea")
(setf (get poi_state_server::GetPOIInfoResponse :datatype-) "poi_state_server/GetPOIInfoResponse")
(setf (get poi_state_server::GetPOIInfoResponse :definition-)
      "string name
---
bool success
string message
poi_state_server/POIInfo info



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



(provide :poi_state_server/GetPOIInfo "c65c76ed817470389da7b2cfd1cdfbea")


