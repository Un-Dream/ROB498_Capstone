
(cl:in-package :asdf)

(defsystem "custom_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AprilTagPosition" :depends-on ("_package_AprilTagPosition"))
    (:file "_package_AprilTagPosition" :depends-on ("_package"))
  ))