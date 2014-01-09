FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/interactive_walk_planner_new/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/replan.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_replan.lisp"
  "../msg_gen/lisp/task_mode.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_task_mode.lisp"
  "../msg_gen/lisp/orientation.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_orientation.lisp"
  "../msg_gen/lisp/foot_sequence.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_foot_sequence.lisp"
  "../msg_gen/lisp/goal_pose.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_goal_pose.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
