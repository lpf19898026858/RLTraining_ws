;; Auto-generated. Do not edit!


(when (boundp 'nlp_drone_control::Plan)
  (if (not (find-package "NLP_DRONE_CONTROL"))
    (make-package "NLP_DRONE_CONTROL"))
  (shadow 'Plan (find-package "NLP_DRONE_CONTROL")))
(unless (find-package "NLP_DRONE_CONTROL::PLAN")
  (make-package "NLP_DRONE_CONTROL::PLAN"))

(in-package "ROS")
;;//! \htmlinclude Plan.msg.html


(defclass nlp_drone_control::Plan
  :super ros::object
  :slots (_tool_calls _replan_mode ))

(defmethod nlp_drone_control::Plan
  (:init
   (&key
    ((:tool_calls __tool_calls) ())
    ((:replan_mode __replan_mode) 0)
    )
   (send-super :init)
   (setq _tool_calls __tool_calls)
   (setq _replan_mode (round __replan_mode))
   self)
  (:tool_calls
   (&rest __tool_calls)
   (if (keywordp (car __tool_calls))
       (send* _tool_calls __tool_calls)
     (progn
       (if __tool_calls (setq _tool_calls (car __tool_calls)))
       _tool_calls)))
  (:replan_mode
   (&optional __replan_mode)
   (if __replan_mode (setq _replan_mode __replan_mode)) _replan_mode)
  (:serialization-length
   ()
   (+
    ;; nlp_drone_control/ToolCall[] _tool_calls
    (apply #'+ (send-all _tool_calls :serialization-length)) 4
    ;; uint8 _replan_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nlp_drone_control/ToolCall[] _tool_calls
     (write-long (length _tool_calls) s)
     (dolist (elem _tool_calls)
       (send elem :serialize s)
       )
     ;; uint8 _replan_mode
       (write-byte _replan_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nlp_drone_control/ToolCall[] _tool_calls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tool_calls (let (r) (dotimes (i n) (push (instance nlp_drone_control::ToolCall :init) r)) r))
     (dolist (elem- _tool_calls)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint8 _replan_mode
     (setq _replan_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get nlp_drone_control::Plan :md5sum-) "6b2a4346febb21fc1800dd9e94fdf8ca")
(setf (get nlp_drone_control::Plan :datatype-) "nlp_drone_control/Plan")
(setf (get nlp_drone_control::Plan :definition-)
      "# Represents a list of tool calls (a full execution plan)
nlp_drone_control/ToolCall[] tool_calls
uint8 replan_mode          # 0 = APPEND, 1 = REPLACE


================================================================================
MSG: nlp_drone_control/ToolCall
# Represents a single tool call produced by the LLM
string function_name       # e.g. \"takeoff\"
string arguments_json      # raw JSON string of arguments


")



(provide :nlp_drone_control/Plan "6b2a4346febb21fc1800dd9e94fdf8ca")


