; Auto-generated. Do not edit!


(cl:in-package rover-msg)


;//! \htmlinclude Enabled.msg.html

(cl:defclass <Enabled> (roslisp-msg-protocol:ros-message)
  ((motorsEnabled
    :reader motorsEnabled
    :initarg :motorsEnabled
    :type cl:boolean
    :initform cl:nil)
   (settingsDumpEnabled
    :reader settingsDumpEnabled
    :initarg :settingsDumpEnabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Enabled (<Enabled>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Enabled>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Enabled)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover-msg:<Enabled> is deprecated: use rover-msg:Enabled instead.")))

(cl:ensure-generic-function 'motorsEnabled-val :lambda-list '(m))
(cl:defmethod motorsEnabled-val ((m <Enabled>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover-msg:motorsEnabled-val is deprecated.  Use rover-msg:motorsEnabled instead.")
  (motorsEnabled m))

(cl:ensure-generic-function 'settingsDumpEnabled-val :lambda-list '(m))
(cl:defmethod settingsDumpEnabled-val ((m <Enabled>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover-msg:settingsDumpEnabled-val is deprecated.  Use rover-msg:settingsDumpEnabled instead.")
  (settingsDumpEnabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Enabled>) ostream)
  "Serializes a message object of type '<Enabled>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motorsEnabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'settingsDumpEnabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Enabled>) istream)
  "Deserializes a message object of type '<Enabled>"
    (cl:setf (cl:slot-value msg 'motorsEnabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'settingsDumpEnabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Enabled>)))
  "Returns string type for a message object of type '<Enabled>"
  "rover/Enabled")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Enabled)))
  "Returns string type for a message object of type 'Enabled"
  "rover/Enabled")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Enabled>)))
  "Returns md5sum for a message object of type '<Enabled>"
  "944b9b58b23f7416adecf8816d3ed0ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Enabled)))
  "Returns md5sum for a message object of type 'Enabled"
  "944b9b58b23f7416adecf8816d3ed0ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Enabled>)))
  "Returns full string definition for message of type '<Enabled>"
  (cl:format cl:nil "bool motorsEnabled~%bool settingsDumpEnabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Enabled)))
  "Returns full string definition for message of type 'Enabled"
  (cl:format cl:nil "bool motorsEnabled~%bool settingsDumpEnabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Enabled>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Enabled>))
  "Converts a ROS message object to a list"
  (cl:list 'Enabled
    (cl:cons ':motorsEnabled (motorsEnabled msg))
    (cl:cons ':settingsDumpEnabled (settingsDumpEnabled msg))
))
