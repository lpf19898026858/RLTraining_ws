
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DoneInfo" :depends-on ("_package_DoneInfo"))
    (:file "_package_DoneInfo" :depends-on ("_package"))
  ))