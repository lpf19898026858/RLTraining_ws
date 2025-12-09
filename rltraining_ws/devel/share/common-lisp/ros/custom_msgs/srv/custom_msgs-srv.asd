
(cl:in-package :asdf)

(defsystem "custom_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ResetEpisode" :depends-on ("_package_ResetEpisode"))
    (:file "_package_ResetEpisode" :depends-on ("_package"))
    (:file "UpdateCurriculum" :depends-on ("_package_UpdateCurriculum"))
    (:file "_package_UpdateCurriculum" :depends-on ("_package"))
  ))