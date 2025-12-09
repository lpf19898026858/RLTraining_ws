;; Auto-generated. Do not edit!


(when (boundp 'nlp_drone_control::Action)
  (if (not (find-package "NLP_DRONE_CONTROL"))
    (make-package "NLP_DRONE_CONTROL"))
  (shadow 'Action (find-package "NLP_DRONE_CONTROL")))
(unless (find-package "NLP_DRONE_CONTROL::ACTION")
  (make-package "NLP_DRONE_CONTROL::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass nlp_drone_control::Action
  :super ros::object
  :slots (_function_name _arguments_json ))

(defmethod nlp_drone_control::Action
  (:init
   (&key
    ((:function_name __function_name) "")
    ((:arguments_json __arguments_json) "")
    )
   (send-super :init)
   (setq _function_name (string __function_name))
   (setq _arguments_json (string __arguments_json))
   self)
  (:function_name
   (&optional __function_name)
   (if __function_name (setq _function_name __function_name)) _function_name)
  (:arguments_json
   (&optional __arguments_json)
   (if __arguments_json (setq _arguments_json __arguments_json)) _arguments_json)
  (:serialization-length
   ()
   (+
    ;; string _function_name
    4 (length _function_name)
    ;; string _arguments_json
    4 (length _arguments_json)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _function_name
       (write-long (length _function_name) s) (princ _function_name s)
     ;; string _arguments_json
       (write-long (length _arguments_json) s) (princ _arguments_json s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _function_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _function_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _arguments_json
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _arguments_json (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get nlp_drone_control::Action :md5sum-) "45955b02be53f69e3133212fb3d15576")
(setf (get nlp_drone_control::Action :datatype-) "nlp_drone_control/Action")
(setf (get nlp_drone_control::Action :definition-)
      "# Represents a single tool call produced by the LLM
string function_name       # e.g. \"takeoff\"
string arguments_json      # raw JSON string of arguments


")



(provide :nlp_drone_control/Action "45955b02be53f69e3133212fb3d15576")


