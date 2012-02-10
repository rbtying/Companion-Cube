; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Settings.msg.html

(cl:defclass <Settings> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_proportional
    :reader left_proportional
    :initarg :left_proportional
    :type cl:float
    :initform 0.0)
   (left_integral
    :reader left_integral
    :initarg :left_integral
    :type cl:float
    :initform 0.0)
   (left_derivative
    :reader left_derivative
    :initarg :left_derivative
    :type cl:float
    :initform 0.0)
   (left_conversion_factor
    :reader left_conversion_factor
    :initarg :left_conversion_factor
    :type cl:float
    :initform 0.0)
   (right_proportional
    :reader right_proportional
    :initarg :right_proportional
    :type cl:float
    :initform 0.0)
   (right_integral
    :reader right_integral
    :initarg :right_integral
    :type cl:float
    :initform 0.0)
   (right_derivative
    :reader right_derivative
    :initarg :right_derivative
    :type cl:float
    :initform 0.0)
   (right_conversion_factor
    :reader right_conversion_factor
    :initarg :right_conversion_factor
    :type cl:float
    :initform 0.0))
)

(cl:defclass Settings (<Settings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Settings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Settings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Settings> is deprecated: use rover_msgs-msg:Settings instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:header-val is deprecated.  Use rover_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_proportional-val :lambda-list '(m))
(cl:defmethod left_proportional-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:left_proportional-val is deprecated.  Use rover_msgs-msg:left_proportional instead.")
  (left_proportional m))

(cl:ensure-generic-function 'left_integral-val :lambda-list '(m))
(cl:defmethod left_integral-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:left_integral-val is deprecated.  Use rover_msgs-msg:left_integral instead.")
  (left_integral m))

(cl:ensure-generic-function 'left_derivative-val :lambda-list '(m))
(cl:defmethod left_derivative-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:left_derivative-val is deprecated.  Use rover_msgs-msg:left_derivative instead.")
  (left_derivative m))

(cl:ensure-generic-function 'left_conversion_factor-val :lambda-list '(m))
(cl:defmethod left_conversion_factor-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:left_conversion_factor-val is deprecated.  Use rover_msgs-msg:left_conversion_factor instead.")
  (left_conversion_factor m))

(cl:ensure-generic-function 'right_proportional-val :lambda-list '(m))
(cl:defmethod right_proportional-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:right_proportional-val is deprecated.  Use rover_msgs-msg:right_proportional instead.")
  (right_proportional m))

(cl:ensure-generic-function 'right_integral-val :lambda-list '(m))
(cl:defmethod right_integral-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:right_integral-val is deprecated.  Use rover_msgs-msg:right_integral instead.")
  (right_integral m))

(cl:ensure-generic-function 'right_derivative-val :lambda-list '(m))
(cl:defmethod right_derivative-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:right_derivative-val is deprecated.  Use rover_msgs-msg:right_derivative instead.")
  (right_derivative m))

(cl:ensure-generic-function 'right_conversion_factor-val :lambda-list '(m))
(cl:defmethod right_conversion_factor-val ((m <Settings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:right_conversion_factor-val is deprecated.  Use rover_msgs-msg:right_conversion_factor instead.")
  (right_conversion_factor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Settings>) ostream)
  "Serializes a message object of type '<Settings>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_proportional))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_integral))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_derivative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_conversion_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_proportional))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_integral))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_derivative))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_conversion_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Settings>) istream)
  "Deserializes a message object of type '<Settings>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_proportional) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_integral) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_derivative) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_conversion_factor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_proportional) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_integral) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_derivative) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_conversion_factor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Settings>)))
  "Returns string type for a message object of type '<Settings>"
  "rover_msgs/Settings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Settings)))
  "Returns string type for a message object of type 'Settings"
  "rover_msgs/Settings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Settings>)))
  "Returns md5sum for a message object of type '<Settings>"
  "c0d59de285a4827280115b3f83474657")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Settings)))
  "Returns md5sum for a message object of type 'Settings"
  "c0d59de285a4827280115b3f83474657")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Settings>)))
  "Returns full string definition for message of type '<Settings>"
  (cl:format cl:nil "Header header~%~%float32 left_proportional~%float32 left_integral~%float32 left_derivative~%float32 left_conversion_factor~%~%float32 right_proportional~%float32 right_integral~%float32 right_derivative~%float32 right_conversion_factor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Settings)))
  "Returns full string definition for message of type 'Settings"
  (cl:format cl:nil "Header header~%~%float32 left_proportional~%float32 left_integral~%float32 left_derivative~%float32 left_conversion_factor~%~%float32 right_proportional~%float32 right_integral~%float32 right_derivative~%float32 right_conversion_factor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Settings>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Settings>))
  "Converts a ROS message object to a list"
  (cl:list 'Settings
    (cl:cons ':header (header msg))
    (cl:cons ':left_proportional (left_proportional msg))
    (cl:cons ':left_integral (left_integral msg))
    (cl:cons ':left_derivative (left_derivative msg))
    (cl:cons ':left_conversion_factor (left_conversion_factor msg))
    (cl:cons ':right_proportional (right_proportional msg))
    (cl:cons ':right_integral (right_integral msg))
    (cl:cons ':right_derivative (right_derivative msg))
    (cl:cons ':right_conversion_factor (right_conversion_factor msg))
))
