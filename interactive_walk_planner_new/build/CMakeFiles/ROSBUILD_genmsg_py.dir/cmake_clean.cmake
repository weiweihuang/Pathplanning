FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/interactive_walk_planner_new/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/interactive_walk_planner_new/msg/__init__.py"
  "../src/interactive_walk_planner_new/msg/_goal_pose.py"
  "../src/interactive_walk_planner_new/msg/_task_mode.py"
  "../src/interactive_walk_planner_new/msg/_foot_sequence.py"
  "../src/interactive_walk_planner_new/msg/_replan.py"
  "../src/interactive_walk_planner_new/msg/_orientation.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
