FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/car_drive_control/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/motion_command.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_motion_command.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
