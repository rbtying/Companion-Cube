FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/neato_node/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/neato_node/msg/__init__.py"
  "../src/neato_node/msg/_BatteryState.py"
  "../src/neato_node/msg/_NeatoDropSensor.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
