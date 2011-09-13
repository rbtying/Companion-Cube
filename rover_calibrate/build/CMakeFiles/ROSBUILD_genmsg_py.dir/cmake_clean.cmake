FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rover_calibrate/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rover_calibrate/msg/__init__.py"
  "../src/rover_calibrate/msg/_ScanAngle.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
