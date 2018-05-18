; Auto-generated. Do not edit!


(cl:in-package rcr2018-msg)


;//! \htmlinclude LineCount.msg.html

(cl:defclass <LineCount> (roslisp-msg-protocol:ros-message)
  ((count
    :reader count
    :initarg :count
    :type cl:fixnum
    :initform 0))
)

(cl:defclass LineCount (<LineCount>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineCount>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineCount)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rcr2018-msg:<LineCount> is deprecated: use rcr2018-msg:LineCount instead.")))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <LineCount>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rcr2018-msg:count-val is deprecated.  Use rcr2018-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineCount>) ostream)
  "Serializes a message object of type '<LineCount>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineCount>) istream)
  "Deserializes a message object of type '<LineCount>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineCount>)))
  "Returns string type for a message object of type '<LineCount>"
  "rcr2018/LineCount")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineCount)))
  "Returns string type for a message object of type 'LineCount"
  "rcr2018/LineCount")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineCount>)))
  "Returns md5sum for a message object of type '<LineCount>"
  "2fa4e861824f4267d0328df36b408141")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineCount)))
  "Returns md5sum for a message object of type 'LineCount"
  "2fa4e861824f4267d0328df36b408141")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineCount>)))
  "Returns full string definition for message of type '<LineCount>"
  (cl:format cl:nil "uint8 count~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineCount)))
  "Returns full string definition for message of type 'LineCount"
  (cl:format cl:nil "uint8 count~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineCount>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineCount>))
  "Converts a ROS message object to a list"
  (cl:list 'LineCount
    (cl:cons ':count (count msg))
))
