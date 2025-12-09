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
  :slots (_actions _replan_mode ))

(defmethod nlp_drone_control::Plan
  (:init
   (&key
    ((:actions __actions) ())
    ((:replan_mode __replan_mode) 0)
    )
   (send-super :init)
   (setq _actions __actions)
   (setq _replan_mode (round __replan_mode))
   self)
  (:actions
   (&rest __actions)
   (if (keywordp (car __actions))
       (send* _actions __actions)
     (progn
       (if __actions (setq _actions (car __actions)))
       _actions)))
  (:replan_mode
   (&optional __replan_mode)
   (if __replan_mode (setq _replan_mode __replan_mode)) _replan_mode)
  (:serialization-length
   ()
   (+
    ;; nlp_drone_control/Action[] _actions
    (apply #'+ (send-all _actions :serialization-length)) 4
    ;; uint8 _replan_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nlp_drone_control/Action[] _actions
     (write-long (length _actions) s)
     (dolist (elem _actions)
       (send elem :serialize s)
       )
     ;; uint8 _replan_mode
       (write-byte _replan_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nlp_drone_control/Action[] _actions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actions (let (r) (dotimes (i n) (push (instance nlp_drone_control::Action :init) r)) r))
     (dolist (elem- _actions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint8 _replan_mode
     (setq _replan_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get nlp_drone_control::Plan :md5sum-) "13f0666e18573937822408eb65c6a4cd")
(setf (get nlp_drone_control::Plan :datatype-) "nlp_drone_control/Plan")
(setf (get nlp_drone_control::Plan :definition-)
      "# Represents a list of tool calls (a full execution plan)
nlp_drone_control/Action[] actions
uint8 replan_mode          # 0 = APPEND, 1 = REPLACE


================================================================================
MSG: nlp_drone_control/Action
# Represents a single tool call produced by the LLM
string function_name       # e.g. \"takeoff\"
string arguments_json      # raw JSON string of arguments


")



(provide :nlp_drone_control/Plan "13f0666e18573937822408eb65c6a4cd")


