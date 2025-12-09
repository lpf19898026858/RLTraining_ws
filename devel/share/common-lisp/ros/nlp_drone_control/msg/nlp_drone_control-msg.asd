
(cl:in-package :asdf)

(defsystem "nlp_drone_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "Plan" :depends-on ("_package_Plan"))
    (:file "_package_Plan" :depends-on ("_package"))
  ))