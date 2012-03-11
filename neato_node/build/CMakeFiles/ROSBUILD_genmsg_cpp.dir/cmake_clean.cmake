FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/neato_node/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/neato_node/BatteryState.h"
  "../msg_gen/cpp/include/neato_node/NeatoDropSensor.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
