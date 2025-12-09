
(cl:in-package :asdf)

(defsystem "nlp_drone_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ExecuteDroneAction" :depends-on ("_package_ExecuteDroneAction"))
    (:file "_package_ExecuteDroneAction" :depends-on ("_package"))
  ))