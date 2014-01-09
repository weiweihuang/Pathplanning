
(cl:in-package :asdf)

(defsystem "interactive_walk_planner_new-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "replan" :depends-on ("_package_replan"))
    (:file "_package_replan" :depends-on ("_package"))
    (:file "task_mode" :depends-on ("_package_task_mode"))
    (:file "_package_task_mode" :depends-on ("_package"))
    (:file "orientation" :depends-on ("_package_orientation"))
    (:file "_package_orientation" :depends-on ("_package"))
    (:file "foot_sequence" :depends-on ("_package_foot_sequence"))
    (:file "_package_foot_sequence" :depends-on ("_package"))
    (:file "goal_pose" :depends-on ("_package_goal_pose"))
    (:file "_package_goal_pose" :depends-on ("_package"))
  ))