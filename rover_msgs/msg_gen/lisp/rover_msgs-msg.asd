
(cl:in-package :asdf)

(defsystem "rover_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Settings" :depends-on ("_package_Settings"))
    (:file "_package_Settings" :depends-on ("_package"))
    (:file "Gyro" :depends-on ("_package_Gyro"))
    (:file "_package_Gyro" :depends-on ("_package"))
    (:file "Motors" :depends-on ("_package_Motors"))
    (:file "_package_Motors" :depends-on ("_package"))
    (:file "Battery" :depends-on ("_package_Battery"))
    (:file "_package_Battery" :depends-on ("_package"))
    (:file "Encoder" :depends-on ("_package_Encoder"))
    (:file "_package_Encoder" :depends-on ("_package"))
    (:file "CondensedIMU" :depends-on ("_package_CondensedIMU"))
    (:file "_package_CondensedIMU" :depends-on ("_package"))
    (:file "Lighting" :depends-on ("_package_Lighting"))
    (:file "_package_Lighting" :depends-on ("_package"))
    (:file "Enabled" :depends-on ("_package_Enabled"))
    (:file "_package_Enabled" :depends-on ("_package"))
  ))