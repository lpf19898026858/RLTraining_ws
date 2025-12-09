
(cl:in-package :asdf)

(defsystem "nlp_drone_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Plan" :depends-on ("_package_Plan"))
    (:file "_package_Plan" :depends-on ("_package"))
    (:file "ToolCall" :depends-on ("_package_ToolCall"))
    (:file "_package_ToolCall" :depends-on ("_package"))
  ))