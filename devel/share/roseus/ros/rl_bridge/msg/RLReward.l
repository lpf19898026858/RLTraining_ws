;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::RLReward)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'RLReward (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::RLREWARD")
  (make-package "RL_BRIDGE::RLREWARD"))

(in-package "ROS")
;;//! \htmlinclude RLReward.msg.html


(defclass rl_bridge::RLReward
  :super ros::object
  :slots (_reward ))

(defmethod rl_bridge::RLReward
  (:init
   (&key
    ((:reward __reward) 0.0)
    )
   (send-super :init)
   (setq _reward (float __reward))
   self)
  (:reward
   (&optional __reward)
   (if __reward (setq _reward __reward)) _reward)
  (:serialization-length
   ()
   (+
    ;; float32 _reward
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _reward
       (sys::poke _reward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _reward
     (setq _reward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rl_bridge::RLReward :md5sum-) "1c5a2bbc277e822b80ec2fb352dd1efe")
(setf (get rl_bridge::RLReward :datatype-) "rl_bridge/RLReward")
(setf (get rl_bridge::RLReward :definition-)
      "float32 reward

")



(provide :rl_bridge/RLReward "1c5a2bbc277e822b80ec2fb352dd1efe")


