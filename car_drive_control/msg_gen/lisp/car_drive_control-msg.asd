
(cl:in-package :asdf)

(defsystem "car_drive_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "motion_command" :depends-on ("_package_motion_command"))
    (:file "_package_motion_command" :depends-on ("_package"))
  ))