
(cl:in-package :asdf)

(defsystem "apriltag2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AprilTagDetectionArray" :depends-on ("_package_AprilTagDetectionArray"))
    (:file "_package_AprilTagDetectionArray" :depends-on ("_package"))
    (:file "AprilTagDetection" :depends-on ("_package_AprilTagDetection"))
    (:file "_package_AprilTagDetection" :depends-on ("_package"))
  ))