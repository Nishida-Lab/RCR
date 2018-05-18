; Auto-generated. Do not edit!


(cl:in-package rcr2018-msg)


;//! \htmlinclude DcmCommand.msg.html

(cl:defclass <DcmCommand> (roslisp-msg-protocol:ros-message)
  ((cmd_vel
    :reader cmd_vel
    :initarg :cmd_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass DcmCommand (<DcmCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DcmCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DcmCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rcr2018-msg:<DcmCommand> is deprecated: use rcr2018-msg:DcmCommand instead.")))

(cl:ensure-generic-function 'cmd_vel-val :lambda-list '(m))
(cl:defmethod cmd_vel-val ((m <DcmCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:cmd_vel-val is deprecated.  Use rcr2018-msg:cmd_vel instead.")
  (cmd_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DcmCommand>) ostream)
  "Serializes a message object of type '<DcmCommand>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cmd_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DcmCommand>) istream)
  "Deserializes a message object of type '<DcmCommand>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cmd_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DcmCommand>)))
  "Returns string type for a message object of type '<DcmCommand>"
  "rcr2018/DcmCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DcmCommand)))
  "Returns string type for a message object of type 'DcmCommand"
  "rcr2018/DcmCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DcmCommand>)))
  "Returns md5sum for a message object of type '<DcmCommand>"
  "93e14900f3ac3cbfca813e5ff5e2bd6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DcmCommand)))
  "Returns md5sum for a message object of type 'DcmCommand"
  "93e14900f3ac3cbfca813e5ff5e2bd6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DcmCommand>)))
  "Returns full string definition for message of type '<DcmCommand>"
  (cl:format cl:nil "float64 cmd_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DcmCommand)))
  "Returns full string definition for message of type 'DcmCommand"
  (cl:format cl:nil "float64 cmd_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DcmCommand>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DcmCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'DcmCommand
    (cl:cons ':cmd_vel (cmd_vel msg))
))
