
(cl:in-package :asdf)

(defsystem "dist_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "robot_data" :depends-on ("_package_robot_data"))
    (:file "_package_robot_data" :depends-on ("_package"))
    (:file "target_assignment_data" :depends-on ("_package_target_assignment_data"))
    (:file "_package_target_assignment_data" :depends-on ("_package"))
    (:file "target_data" :depends-on ("_package_target_data"))
    (:file "_package_target_data" :depends-on ("_package"))
    (:file "targets_data" :depends-on ("_package_targets_data"))
    (:file "_package_targets_data" :depends-on ("_package"))
    (:file "uwb_data" :depends-on ("_package_uwb_data"))
    (:file "_package_uwb_data" :depends-on ("_package"))
  ))