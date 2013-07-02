
(cl:in-package :asdf)

(defsystem "manipulation_transforms-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "EffectorTrajectories" :depends-on ("_package_EffectorTrajectories"))
    (:file "_package_EffectorTrajectories" :depends-on ("_package"))
    (:file "EffectorTrajectoriesPoint" :depends-on ("_package_EffectorTrajectoriesPoint"))
    (:file "_package_EffectorTrajectoriesPoint" :depends-on ("_package"))
  ))