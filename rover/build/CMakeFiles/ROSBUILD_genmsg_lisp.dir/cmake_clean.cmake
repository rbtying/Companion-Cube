FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rover/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Settings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Settings.lisp"
  "../msg_gen/lisp/Gyro.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Gyro.lisp"
  "../msg_gen/lisp/Motors.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Motors.lisp"
  "../msg_gen/lisp/Battery.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Battery.lisp"
  "../msg_gen/lisp/Encoder.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Encoder.lisp"
  "../msg_gen/lisp/CondensedIMU.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_CondensedIMU.lisp"
  "../msg_gen/lisp/Enabled.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Enabled.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
