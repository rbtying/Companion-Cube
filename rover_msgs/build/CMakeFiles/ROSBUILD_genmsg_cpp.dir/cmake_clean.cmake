FILE(REMOVE_RECURSE
  "../src/rover_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rover_msgs/Settings.h"
  "../msg_gen/cpp/include/rover_msgs/Enabled.h"
  "../msg_gen/cpp/include/rover_msgs/Gyro.h"
  "../msg_gen/cpp/include/rover_msgs/CondensedIMU.h"
  "../msg_gen/cpp/include/rover_msgs/Battery.h"
  "../msg_gen/cpp/include/rover_msgs/Encoder.h"
  "../msg_gen/cpp/include/rover_msgs/Motors.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
