;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::RLAction)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'RLAction (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::RLACTION")
  (make-package "RL_BRIDGE::RLACTION"))

(in-package "ROS")
;;//! \htmlinclude RLAction.msg.html


(defclass rl_bridge::RLAction
  :super ros::object
  :slots (_ax _ay ))

(defmethod rl_bridge::RLAction
  (:init
   (&key
    ((:ax __ax) 0.0)
    ((:ay __ay) 0.0)
    )
   (send-super :init)
   (setq _ax (float __ax))
   (setq _ay (float __ay))
   self)
  (:ax
   (&optional __ax)
   (if __ax (setq _ax __ax)) _ax)
  (:ay
   (&optional __ay)
   (if __ay (setq _ay __ay)) _ay)
  (:serialization-length
   ()
   (+
    ;; float32 _ax
    4
    ;; float32 _ay
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ax
       (sys::poke _ax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ay
       (sys::poke _ay (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ax
     (setq _ax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ay
     (setq _ay (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rl_bridge::RLAction :md5sum-) "4eb4e8d132b979f0df69e04e0a241d57")
(setf (get rl_bridge::RLAction :datatype-) "rl_bridge/RLAction")
(setf (get rl_bridge::RLAction :definition-)
      "float32 ax
float32 ay


")



(provide :rl_bridge/RLAction "4eb4e8d132b979f0df69e04e0a241d57")


