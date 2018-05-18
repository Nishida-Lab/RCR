; Auto-generated. Do not edit!


(cl:in-package rcr2018-msg)


;//! \htmlinclude TofSide.msg.html

(cl:defclass <TofSide> (roslisp-msg-protocol:ros-message)
  ((right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0)
   (left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0))
)

(cl:defclass TofSide (<TofSide>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TofSide>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TofSide)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rcr2018-msg:<TofSide> is deprecated: use rcr2018-msg:TofSide instead.")))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <TofSide>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:right-val is deprecated.  Use rcr2018-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <TofSide>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:left-val is deprecated.  Use rcr2018-msg:left instead.")
  (left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TofSide>) ostream)
  "Serializes a message object of type '<TofSide>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TofSide>) istream)
  "Deserializes a message object of type '<TofSide>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TofSide>)))
  "Returns string type for a message object of type '<TofSide>"
  "rcr2018/TofSide")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TofSide)))
  "Returns string type for a message object of type 'TofSide"
  "rcr2018/TofSide")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TofSide>)))
  "Returns md5sum for a message object of type '<TofSide>"
  "69961745475ae50d2f0b8ca4d0df065b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TofSide)))
  "Returns md5sum for a message object of type 'TofSide"
  "69961745475ae50d2f0b8ca4d0df065b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TofSide>)))
  "Returns full string definition for message of type '<TofSide>"
  (cl:format cl:nil "float64 right~%float64 left~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TofSide)))
  "Returns full string definition for message of type 'TofSide"
  (cl:format cl:nil "float64 right~%float64 left~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TofSide>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TofSide>))
  "Converts a ROS message object to a list"
  (cl:list 'TofSide
    (cl:cons ':right (right msg))
    (cl:cons ':left (left msg))
))
