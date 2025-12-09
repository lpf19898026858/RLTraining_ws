;; Auto-generated. Do not edit!


(when (boundp 'nlp_drone_control::ExecuteDroneAction)
  (if (not (find-package "NLP_DRONE_CONTROL"))
    (make-package "NLP_DRONE_CONTROL"))
  (shadow 'ExecuteDroneAction (find-package "NLP_DRONE_CONTROL")))
(unless (find-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTION")
  (make-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTION"))
(unless (find-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTIONREQUEST")
  (make-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTIONREQUEST"))
(unless (find-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTIONRESPONSE")
  (make-package "NLP_DRONE_CONTROL::EXECUTEDRONEACTIONRESPONSE"))

(in-package "ROS")





(defclass nlp_drone_control::ExecuteDroneActionRequest
  :super ros::object
  :slots (_action_type _params_json ))

(defmethod nlp_drone_control::ExecuteDroneActionRequest
  (:init
   (&key
    ((:action_type __action_type) "")
    ((:params_json __params_json) "")
    )
   (send-super :init)
   (setq _action_type (string __action_type))
   (setq _params_json (string __params_json))
   self)
  (:action_type
   (&optional __action_type)
   (if __action_type (setq _action_type __action_type)) _action_type)
  (:params_json
   (&optional __params_json)
   (if __params_json (setq _params_json __params_json)) _params_json)
  (:serialization-length
   ()
   (+
    ;; string _action_type
    4 (length _action_type)
    ;; string _params_json
    4 (length _params_json)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action_type
       (write-long (length _action_type) s) (princ _action_type s)
     ;; string _params_json
       (write-long (length _params_json) s) (princ _params_json s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _params_json
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _params_json (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass nlp_drone_control::ExecuteDroneActionResponse
  :super ros::object
  :slots (_success _message ))

(defmethod nlp_drone_control::ExecuteDroneActionResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass nlp_drone_control::ExecuteDroneAction
  :super ros::object
  :slots ())

(setf (get nlp_drone_control::ExecuteDroneAction :md5sum-) "77584cf99b7cb35fa13143d3f49ac104")
(setf (get nlp_drone_control::ExecuteDroneAction :datatype-) "nlp_drone_control/ExecuteDroneAction")
(setf (get nlp_drone_control::ExecuteDroneAction :request) nlp_drone_control::ExecuteDroneActionRequest)
(setf (get nlp_drone_control::ExecuteDroneAction :response) nlp_drone_control::ExecuteDroneActionResponse)

(defmethod nlp_drone_control::ExecuteDroneActionRequest
  (:response () (instance nlp_drone_control::ExecuteDroneActionResponse :init)))

(setf (get nlp_drone_control::ExecuteDroneActionRequest :md5sum-) "77584cf99b7cb35fa13143d3f49ac104")
(setf (get nlp_drone_control::ExecuteDroneActionRequest :datatype-) "nlp_drone_control/ExecuteDroneActionRequest")
(setf (get nlp_drone_control::ExecuteDroneActionRequest :definition-)
      "# --- Request ---
# 动作的类型，例如 \"adjust_camera\", \"rotate_360\", \"hover_and_monitor\"
string action_type

# (可选) 动作的参数，我们可以用一个JSON字符串来灵活地传递
# 例如：对于adjust_camera, params可以是 \"{\\\"pitch_angle\\\": -90.0}\"
# 对于hover_and_monitor, params可以是 \"{\\\"duration_seconds\\\": 5.0}\"
string params_json

---
# --- Response ---
bool success          # 动作是否成功执行
string message        # 返回给LLM的信息，例如 \"Camera adjusted.\" 或 \"Rotation failed.\"

")

(setf (get nlp_drone_control::ExecuteDroneActionResponse :md5sum-) "77584cf99b7cb35fa13143d3f49ac104")
(setf (get nlp_drone_control::ExecuteDroneActionResponse :datatype-) "nlp_drone_control/ExecuteDroneActionResponse")
(setf (get nlp_drone_control::ExecuteDroneActionResponse :definition-)
      "# --- Request ---
# 动作的类型，例如 \"adjust_camera\", \"rotate_360\", \"hover_and_monitor\"
string action_type

# (可选) 动作的参数，我们可以用一个JSON字符串来灵活地传递
# 例如：对于adjust_camera, params可以是 \"{\\\"pitch_angle\\\": -90.0}\"
# 对于hover_and_monitor, params可以是 \"{\\\"duration_seconds\\\": 5.0}\"
string params_json

---
# --- Response ---
bool success          # 动作是否成功执行
string message        # 返回给LLM的信息，例如 \"Camera adjusted.\" 或 \"Rotation failed.\"

")



(provide :nlp_drone_control/ExecuteDroneAction "77584cf99b7cb35fa13143d3f49ac104")


