FILE(REMOVE_RECURSE
  "../src/rover/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rover/Settings.h"
  "../msg_gen/cpp/include/rover/Enabled.h"
  "../msg_gen/cpp/include/rover/Gyro.h"
  "../msg_gen/cpp/include/rover/CondensedIMU.h"
  "../msg_gen/cpp/include/rover/Battery.h"
  "../msg_gen/cpp/include/rover/Encoder.h"
  "../msg_gen/cpp/include/rover/Motors.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
