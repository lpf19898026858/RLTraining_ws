;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::DoneInfo)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'DoneInfo (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::DONEINFO")
  (make-package "CUSTOM_MSGS::DONEINFO"))

(in-package "ROS")
;;//! \htmlinclude DoneInfo.msg.html


(defclass custom_msgs::DoneInfo
  :super ros::object
  :slots (_is_done _is_success _reason ))

(defmethod custom_msgs::DoneInfo
  (:init
   (&key
    ((:is_done __is_done) nil)
    ((:is_success __is_success) nil)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _is_done __is_done)
   (setq _is_success __is_success)
   (setq _reason (string __reason))
   self)
  (:is_done
   (&optional (__is_done :null))
   (if (not (eq __is_done :null)) (setq _is_done __is_done)) _is_done)
  (:is_success
   (&optional (__is_success :null))
   (if (not (eq __is_success :null)) (setq _is_success __is_success)) _is_success)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; bool _is_done
    1
    ;; bool _is_success
    1
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_done
       (if _is_done (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_success
       (if _is_success (write-byte -1 s) (write-byte 0 s))
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_done
     (setq _is_done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_success
     (setq _is_success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get custom_msgs::DoneInfo :md5sum-) "f554d70bf7374158f003bce0d316e6fd")
(setf (get custom_msgs::DoneInfo :datatype-) "custom_msgs/DoneInfo")
(setf (get custom_msgs::DoneInfo :definition-)
      "bool is_done
bool is_success
string reason

")



(provide :custom_msgs/DoneInfo "f554d70bf7374158f003bce0d316e6fd")


