;; Auto-generated. Do not edit!


(when (boundp 'rl_bridge::RLEvent)
  (if (not (find-package "RL_BRIDGE"))
    (make-package "RL_BRIDGE"))
  (shadow 'RLEvent (find-package "RL_BRIDGE")))
(unless (find-package "RL_BRIDGE::RLEVENT")
  (make-package "RL_BRIDGE::RLEVENT"))

(in-package "ROS")
;;//! \htmlinclude RLEvent.msg.html


(defclass rl_bridge::RLEvent
  :super ros::object
  :slots (_event_type _data _done ))

(defmethod rl_bridge::RLEvent
  (:init
   (&key
    ((:event_type __event_type) "")
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:done __done) nil)
    )
   (send-super :init)
   (setq _event_type (string __event_type))
   (setq _data __data)
   (setq _done __done)
   self)
  (:event_type
   (&optional __event_type)
   (if __event_type (setq _event_type __event_type)) _event_type)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:done
   (&optional (__done :null))
   (if (not (eq __done :null)) (setq _done __done)) _done)
  (:serialization-length
   ()
   (+
    ;; string _event_type
    4 (length _event_type)
    ;; float32[] _data
    (* 4    (length _data)) 4
    ;; bool _done
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _event_type
       (write-long (length _event_type) s) (princ _event_type s)
     ;; float32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _done
       (if _done (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _event_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _event_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _done
     (setq _done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rl_bridge::RLEvent :md5sum-) "3d11c09bdebe9d87801e4c84e2840393")
(setf (get rl_bridge::RLEvent :datatype-) "rl_bridge/RLEvent")
(setf (get rl_bridge::RLEvent :definition-)
      "# RLEvent.msg

string event_type         # 事件类型，例如 \"move\", \"reached_target\", \"collision\", \"out_of_bounds\"
float32[] data            # 附带数据，例如 [distance, velocity, angle]
bool done                 # 是否终止当前 episode


")



(provide :rl_bridge/RLEvent "3d11c09bdebe9d87801e4c84e2840393")


