; Auto-generated. Do not edit!


(cl:in-package rcr2018-msg)


;//! \htmlinclude AngVel.msg.html

(cl:defclass <AngVel> (roslisp-msg-protocol:ros-message)
  ((ang_vel
    :reader ang_vel
    :initarg :ang_vel
    :type cl:float
    :initform 0.0))
)

(cl:defclass AngVel (<AngVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rcr2018-msg:<AngVel> is deprecated: use rcr2018-msg:AngVel instead.")))

(cl:ensure-generic-function 'ang_vel-val :lambda-list '(m))
(cl:defmethod ang_vel-val ((m <AngVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:ang_vel-val is deprecated.  Use rcr2018-msg:ang_vel instead.")
  (ang_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngVel>) ostream)
  "Serializes a message object of type '<AngVel>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ang_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngVel>) istream)
  "Deserializes a message object of type '<AngVel>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang_vel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngVel>)))
  "Returns string type for a message object of type '<AngVel>"
  "rcr2018/AngVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngVel)))
  "Returns string type for a message object of type 'AngVel"
  "rcr2018/AngVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngVel>)))
  "Returns md5sum for a message object of type '<AngVel>"
  "0654aa42e34bb45bcf83b5431e81941d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngVel)))
  "Returns md5sum for a message object of type 'AngVel"
  "0654aa42e34bb45bcf83b5431e81941d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngVel>)))
  "Returns full string definition for message of type '<AngVel>"
  (cl:format cl:nil "float64 ang_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngVel)))
  "Returns full string definition for message of type 'AngVel"
  (cl:format cl:nil "float64 ang_vel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngVel>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngVel>))
  "Converts a ROS message object to a list"
  (cl:list 'AngVel
    (cl:cons ':ang_vel (ang_vel msg))
))
