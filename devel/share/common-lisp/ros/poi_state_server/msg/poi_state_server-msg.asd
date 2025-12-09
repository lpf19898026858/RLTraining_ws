
(cl:in-package :asdf)

(defsystem "poi_state_server-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "NamedPoint" :depends-on ("_package_NamedPoint"))
    (:file "_package_NamedPoint" :depends-on ("_package"))
    (:file "POIInfo" :depends-on ("_package_POIInfo"))
    (:file "_package_POIInfo" :depends-on ("_package"))
  ))