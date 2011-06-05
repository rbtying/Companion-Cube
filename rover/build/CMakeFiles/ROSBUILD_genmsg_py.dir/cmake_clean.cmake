FILE(REMOVE_RECURSE
  "../src/rover/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rover/msg/__init__.py"
  "../src/rover/msg/_Battery.py"
  "../src/rover/msg/_Encoder.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
