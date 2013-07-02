
(cl:in-package :asdf)

(defsystem "cart_local_planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pose2D" :depends-on ("_package_pose2D"))
    (:file "_package_pose2D" :depends-on ("_package"))
  ))