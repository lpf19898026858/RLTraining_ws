
(cl:in-package :asdf)

(defsystem "fleet_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :fleet_manager-msg
)
  :components ((:file "_package")
    (:file "InjectTasks" :depends-on ("_package_InjectTasks"))
    (:file "_package_InjectTasks" :depends-on ("_package"))
    (:file "QueryFleetAndTasks" :depends-on ("_package_QueryFleetAndTasks"))
    (:file "_package_QueryFleetAndTasks" :depends-on ("_package"))
  ))