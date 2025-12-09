
(cl:in-package :asdf)

(defsystem "fleet_manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nlp_drone_control-msg
)
  :components ((:file "_package")
    (:file "DroneStatus" :depends-on ("_package_DroneStatus"))
    (:file "_package_DroneStatus" :depends-on ("_package"))
    (:file "TaskStatus" :depends-on ("_package_TaskStatus"))
    (:file "_package_TaskStatus" :depends-on ("_package"))
  ))