
(cl:in-package :asdf)

(defsystem "neato_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BatteryState" :depends-on ("_package_BatteryState"))
    (:file "_package_BatteryState" :depends-on ("_package"))
    (:file "NeatoDropSensor" :depends-on ("_package_NeatoDropSensor"))
    (:file "_package_NeatoDropSensor" :depends-on ("_package"))
  ))