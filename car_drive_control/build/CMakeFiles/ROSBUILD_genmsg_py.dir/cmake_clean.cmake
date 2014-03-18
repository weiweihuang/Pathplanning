FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/car_drive_control/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/car_drive_control/msg/__init__.py"
  "../src/car_drive_control/msg/_motion_command.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
