FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rover/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rover/msg/__init__.py"
  "../src/rover/msg/_Encoder.py"
  "../src/rover/msg/_Battery.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
