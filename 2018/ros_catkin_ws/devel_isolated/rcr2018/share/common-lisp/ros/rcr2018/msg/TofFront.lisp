; Auto-generated. Do not edit!


(cl:in-package rcr2018-msg)


;//! \htmlinclude TofFront.msg.html

(cl:defclass <TofFront> (roslisp-msg-protocol:ros-message)
  ((front
    :reader front
    :initarg :front
    :type cl:float
    :initform 0.0))
)

(cl:defclass TofFront (<TofFront>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TofFront>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TofFront)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rcr2018-msg:<TofFront> is deprecated: use rcr2018-msg:TofFront instead.")))

(cl:ensure-generic-function 'front-val :lambda-list '(m))
(cl:defmethod front-val ((m <TofFront>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:front-val is deprecated.  Use rcr2018-msg:front instead.")
  (front m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TofFront>) ostream)
  "Serializes a message object of type '<TofFront>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TofFront>) istream)
  "Deserializes a message object of type '<TofFront>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TofFront>)))
  "Returns string type for a message object of type '<TofFront>"
  "rcr2018/TofFront")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TofFront)))
  "Returns string type for a message object of type 'TofFront"
  "rcr2018/TofFront")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TofFront>)))
  "Returns md5sum for a message object of type '<TofFront>"
  "ea4f19e94f2ce3074d518ff4d026a5c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TofFront)))
  "Returns md5sum for a message object of type 'TofFront"
  "ea4f19e94f2ce3074d518ff4d026a5c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TofFront>)))
  "Returns full string definition for message of type '<TofFront>"
  (cl:format cl:nil "float64 front~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TofFront)))
  "Returns full string definition for message of type 'TofFront"
  (cl:format cl:nil "float64 front~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TofFront>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TofFront>))
  "Converts a ROS message object to a list"
  (cl:list 'TofFront
    (cl:cons ':front (front msg))
))
