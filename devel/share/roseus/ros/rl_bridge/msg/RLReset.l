;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::RLReset)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'RLReset (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::RLRESET")
  (make-package "RL_BRIDGE::RLRESET"))

(in-package "ROS")
;;//! \htmlinclude RLReset.msg.html


(defclass rl_bridge::RLReset
  :super ros::object
  :slots (_reset ))

(defmethod rl_bridge::RLReset
  (:init
   (&key
    ((:reset __reset) nil)
    )
   (send-super :init)
   (setq _reset __reset)
   self)
  (:reset
   (&optional (__reset :null))
   (if (not (eq __reset :null)) (setq _reset __reset)) _reset)
  (:serialization-length
   ()
   (+
    ;; bool _reset
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _reset
       (if _reset (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _reset
     (setq _reset (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rl_bridge::RLReset :md5sum-) "ba4b0b221fb425ac5eaf73f71ae34971")
(setf (get rl_bridge::RLReset :datatype-) "rl_bridge/RLReset")
(setf (get rl_bridge::RLReset :definition-)
      "bool reset

")



(provide :rl_bridge/RLReset "ba4b0b221fb425ac5eaf73f71ae34971")


