FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/neato_node/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/BatteryState.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_BatteryState.lisp"
  "../msg_gen/lisp/NeatoDropSensor.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_NeatoDropSensor.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
