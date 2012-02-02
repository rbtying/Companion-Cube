FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rover/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rover/msg/__init__.py"
  "../src/rover/msg/_Settings.py"
  "../src/rover/msg/_Gyro.py"
  "../src/rover/msg/_Motors.py"
  "../src/rover/msg/_Battery.py"
  "../src/rover/msg/_Encoder.py"
  "../src/rover/msg/_CondensedIMU.py"
  "../src/rover/msg/_Enabled.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
