
(cl:in-package :asdf)

(defsystem "vlm_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "CaptureImage" :depends-on ("_package_CaptureImage"))
    (:file "_package_CaptureImage" :depends-on ("_package"))
    (:file "DescribeScene" :depends-on ("_package_DescribeScene"))
    (:file "_package_DescribeScene" :depends-on ("_package"))
  ))