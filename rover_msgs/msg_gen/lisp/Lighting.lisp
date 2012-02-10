; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Lighting.msg.html

(cl:defclass <Lighting> (roslisp-msg-protocol:ros-message)
  ((red_duty_cycle
    :reader red_duty_cycle
    :initarg :red_duty_cycle
    :type cl:float
    :initform 0.0)
   (red_current
    :reader red_current
    :initarg :red_current
    :type cl:float
    :initform 0.0)
   (green_duty_cycle
    :reader green_duty_cycle
    :initarg :green_duty_cycle
    :type cl:float
    :initform 0.0)
   (green_current
    :reader green_current
    :initarg :green_current
    :type cl:float
    :initform 0.0)
   (blue_duty_cycle
    :reader blue_duty_cycle
    :initarg :blue_duty_cycle
    :type cl:float
    :initform 0.0)
   (blue_current
    :reader blue_current
    :initarg :blue_current
    :type cl:float
    :initform 0.0))
)

(cl:defclass Lighting (<Lighting>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lighting>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lighting)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Lighting> is deprecated: use rover_msgs-msg:Lighting instead.")))

(cl:ensure-generic-function 'red_duty_cycle-val :lambda-list '(m))
(cl:defmethod red_duty_cycle-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:red_duty_cycle-val is deprecated.  Use rover_msgs-msg:red_duty_cycle instead.")
  (red_duty_cycle m))

(cl:ensure-generic-function 'red_current-val :lambda-list '(m))
(cl:defmethod red_current-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:red_current-val is deprecated.  Use rover_msgs-msg:red_current instead.")
  (red_current m))

(cl:ensure-generic-function 'green_duty_cycle-val :lambda-list '(m))
(cl:defmethod green_duty_cycle-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:green_duty_cycle-val is deprecated.  Use rover_msgs-msg:green_duty_cycle instead.")
  (green_duty_cycle m))

(cl:ensure-generic-function 'green_current-val :lambda-list '(m))
(cl:defmethod green_current-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:green_current-val is deprecated.  Use rover_msgs-msg:green_current instead.")
  (green_current m))

(cl:ensure-generic-function 'blue_duty_cycle-val :lambda-list '(m))
(cl:defmethod blue_duty_cycle-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:blue_duty_cycle-val is deprecated.  Use rover_msgs-msg:blue_duty_cycle instead.")
  (blue_duty_cycle m))

(cl:ensure-generic-function 'blue_current-val :lambda-list '(m))
(cl:defmethod blue_current-val ((m <Lighting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:blue_current-val is deprecated.  Use rover_msgs-msg:blue_current instead.")
  (blue_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lighting>) ostream)
  "Serializes a message object of type '<Lighting>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'red_duty_cycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'red_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'green_duty_cycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'green_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'blue_duty_cycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'blue_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lighting>) istream)
  "Deserializes a message object of type '<Lighting>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'red_duty_cycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'red_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'green_duty_cycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'green_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blue_duty_cycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'blue_current) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lighting>)))
  "Returns string type for a message object of type '<Lighting>"
  "rover_msgs/Lighting")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lighting)))
  "Returns string type for a message object of type 'Lighting"
  "rover_msgs/Lighting")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lighting>)))
  "Returns md5sum for a message object of type '<Lighting>"
  "8466f8703abc19a467a2579c653659e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lighting)))
  "Returns md5sum for a message object of type 'Lighting"
  "8466f8703abc19a467a2579c653659e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lighting>)))
  "Returns full string definition for message of type '<Lighting>"
  (cl:format cl:nil "float32 red_duty_cycle~%float32 red_current~%float32 green_duty_cycle~%float32 green_current~%float32 blue_duty_cycle~%float32 blue_current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lighting)))
  "Returns full string definition for message of type 'Lighting"
  (cl:format cl:nil "float32 red_duty_cycle~%float32 red_current~%float32 green_duty_cycle~%float32 green_current~%float32 blue_duty_cycle~%float32 blue_current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lighting>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lighting>))
  "Converts a ROS message object to a list"
  (cl:list 'Lighting
    (cl:cons ':red_duty_cycle (red_duty_cycle msg))
    (cl:cons ':red_current (red_current msg))
    (cl:cons ':green_duty_cycle (green_duty_cycle msg))
    (cl:cons ':green_current (green_current msg))
    (cl:cons ':blue_duty_cycle (blue_duty_cycle msg))
    (cl:cons ':blue_current (blue_current msg))
))
