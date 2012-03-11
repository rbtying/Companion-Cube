; Auto-generated. Do not edit!


(cl:in-package neato_node-msg)


;//! \htmlinclude NeatoDropSensor.msg.html

(cl:defclass <NeatoDropSensor> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left
    :reader left
    :initarg :left
    :type cl:fixnum
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:fixnum
    :initform 0))
)

(cl:defclass NeatoDropSensor (<NeatoDropSensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NeatoDropSensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NeatoDropSensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neato_node-msg:<NeatoDropSensor> is deprecated: use neato_node-msg:NeatoDropSensor instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NeatoDropSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neato_node-msg:header-val is deprecated.  Use neato_node-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <NeatoDropSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neato_node-msg:left-val is deprecated.  Use neato_node-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <NeatoDropSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neato_node-msg:right-val is deprecated.  Use neato_node-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NeatoDropSensor>) ostream)
  "Serializes a message object of type '<NeatoDropSensor>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NeatoDropSensor>) istream)
  "Deserializes a message object of type '<NeatoDropSensor>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NeatoDropSensor>)))
  "Returns string type for a message object of type '<NeatoDropSensor>"
  "neato_node/NeatoDropSensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NeatoDropSensor)))
  "Returns string type for a message object of type 'NeatoDropSensor"
  "neato_node/NeatoDropSensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NeatoDropSensor>)))
  "Returns md5sum for a message object of type '<NeatoDropSensor>"
  "efa75ea3c521053e0d014dffae46baa2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NeatoDropSensor)))
  "Returns md5sum for a message object of type 'NeatoDropSensor"
  "efa75ea3c521053e0d014dffae46baa2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NeatoDropSensor>)))
  "Returns full string definition for message of type '<NeatoDropSensor>"
  (cl:format cl:nil "Header header~%uint16 left~%uint16 right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NeatoDropSensor)))
  "Returns full string definition for message of type 'NeatoDropSensor"
  (cl:format cl:nil "Header header~%uint16 left~%uint16 right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NeatoDropSensor>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NeatoDropSensor>))
  "Converts a ROS message object to a list"
  (cl:list 'NeatoDropSensor
    (cl:cons ':header (header msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
