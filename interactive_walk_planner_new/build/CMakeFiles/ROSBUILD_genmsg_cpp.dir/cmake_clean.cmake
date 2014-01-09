FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/interactive_walk_planner_new/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/interactive_walk_planner_new/replan.h"
  "../msg_gen/cpp/include/interactive_walk_planner_new/task_mode.h"
  "../msg_gen/cpp/include/interactive_walk_planner_new/orientation.h"
  "../msg_gen/cpp/include/interactive_walk_planner_new/foot_sequence.h"
  "../msg_gen/cpp/include/interactive_walk_planner_new/goal_pose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
