;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::RLDone)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'RLDone (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::RLDONE")
  (make-package "RL_BRIDGE::RLDONE"))

(in-package "ROS")
;;//! \htmlinclude RLDone.msg.html


(defclass rl_bridge::RLDone
  :super ros::object
  :slots (_done ))

(defmethod rl_bridge::RLDone
  (:init
   (&key
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _done __done)
   self)
  (:done
   (&optional (__done :null))
   (if (not (eq __done :null)) (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rl_bridge::RLDone :md5sum-) "89bb254424e4cffedbf494e7b0ddbfea")
(setf (get rl_bridge::RLDone :datatype-) "rl_bridge/RLDone")
(setf (get rl_bridge::RLDone :definition-)
      "bool done

")



(provide :rl_bridge/RLDone "89bb254424e4cffedbf494e7b0ddbfea")


