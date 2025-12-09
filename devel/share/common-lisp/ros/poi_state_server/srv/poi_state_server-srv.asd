
(cl:in-package :asdf)

(defsystem "poi_state_server-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :poi_state_server-msg
)
  :components ((:file "_package")
    (:file "GetPOIInfo" :depends-on ("_package_GetPOIInfo"))
    (:file "_package_GetPOIInfo" :depends-on ("_package"))
    (:file "ListPOIs" :depends-on ("_package_ListPOIs"))
    (:file "_package_ListPOIs" :depends-on ("_package"))
  ))