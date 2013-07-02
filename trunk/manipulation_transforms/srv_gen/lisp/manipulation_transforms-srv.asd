
(cl:in-package :asdf)

(defsystem "manipulation_transforms-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :manipulation_transforms-msg
)
  :components ((:file "_package")
    (:file "MapObjectPoseToEffectors" :depends-on ("_package_MapObjectPoseToEffectors"))
    (:file "_package_MapObjectPoseToEffectors" :depends-on ("_package"))
    (:file "LoadInitialTransforms" :depends-on ("_package_LoadInitialTransforms"))
    (:file "_package_LoadInitialTransforms" :depends-on ("_package"))
    (:file "MapEffectorTrajectoriesToObject" :depends-on ("_package_MapEffectorTrajectoriesToObject"))
    (:file "_package_MapEffectorTrajectoriesToObject" :depends-on ("_package"))
    (:file "SetInitialTransforms" :depends-on ("_package_SetInitialTransforms"))
    (:file "_package_SetInitialTransforms" :depends-on ("_package"))
    (:file "MapObjectTrajectoryToEffectors" :depends-on ("_package_MapObjectTrajectoryToEffectors"))
    (:file "_package_MapObjectTrajectoryToEffectors" :depends-on ("_package"))
    (:file "MapEffectorTwistsToObject" :depends-on ("_package_MapEffectorTwistsToObject"))
    (:file "_package_MapEffectorTwistsToObject" :depends-on ("_package"))
    (:file "MapEffectorPosesToObject" :depends-on ("_package_MapEffectorPosesToObject"))
    (:file "_package_MapEffectorPosesToObject" :depends-on ("_package"))
    (:file "MapObjectTwistToEffectors" :depends-on ("_package_MapObjectTwistToEffectors"))
    (:file "_package_MapObjectTwistToEffectors" :depends-on ("_package"))
  ))