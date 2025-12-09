
(cl:in-package :asdf)

(defsystem "uav_scheduler-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ComputeAssignment" :depends-on ("_package_ComputeAssignment"))
    (:file "_package_ComputeAssignment" :depends-on ("_package"))
  ))