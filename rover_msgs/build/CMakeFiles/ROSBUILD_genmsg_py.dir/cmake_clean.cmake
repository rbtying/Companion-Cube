FILE(REMOVE_RECURSE
  "../src/rover_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rover_msgs/msg/__init__.py"
  "../src/rover_msgs/msg/_Settings.py"
  "../src/rover_msgs/msg/_Enabled.py"
  "../src/rover_msgs/msg/_Gyro.py"
  "../src/rover_msgs/msg/_CondensedIMU.py"
  "../src/rover_msgs/msg/_Battery.py"
  "../src/rover_msgs/msg/_Encoder.py"
  "../src/rover_msgs/msg/_Motors.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
