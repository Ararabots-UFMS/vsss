; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude motor_speed.msg.html

(cl:defclass <motor_speed> (roslisp-msg-protocol:ros-message)
  ((vel_l
    :reader vel_l
    :initarg :vel_l
    :type cl:integer
    :initform 0)
   (vel_r
    :reader vel_r
    :initarg :vel_r
    :type cl:integer
    :initform 0))
)

(cl:defclass motor_speed (<motor_speed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_speed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_speed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<motor_speed> is deprecated: use verysmall-msg:motor_speed instead.")))

(cl:ensure-generic-function 'vel_l-val :lambda-list '(m))
(cl:defmethod vel_l-val ((m <motor_speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:vel_l-val is deprecated.  Use verysmall-msg:vel_l instead.")
  (vel_l m))

(cl:ensure-generic-function 'vel_r-val :lambda-list '(m))
(cl:defmethod vel_r-val ((m <motor_speed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:vel_r-val is deprecated.  Use verysmall-msg:vel_r instead.")
  (vel_r m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_speed>) ostream)
  "Serializes a message object of type '<motor_speed>"
  (cl:let* ((signed (cl:slot-value msg 'vel_l)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vel_r)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_speed>) istream)
  "Deserializes a message object of type '<motor_speed>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel_l) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel_r) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_speed>)))
  "Returns string type for a message object of type '<motor_speed>"
  "verysmall/motor_speed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_speed)))
  "Returns string type for a message object of type 'motor_speed"
  "verysmall/motor_speed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_speed>)))
  "Returns md5sum for a message object of type '<motor_speed>"
  "f2dd34999ee8b444344eaa77fc91500c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_speed)))
  "Returns md5sum for a message object of type 'motor_speed"
  "f2dd34999ee8b444344eaa77fc91500c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_speed>)))
  "Returns full string definition for message of type '<motor_speed>"
  (cl:format cl:nil "int32            vel_l~%int32            vel_r~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_speed)))
  "Returns full string definition for message of type 'motor_speed"
  (cl:format cl:nil "int32            vel_l~%int32            vel_r~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_speed>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_speed>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_speed
    (cl:cons ':vel_l (vel_l msg))
    (cl:cons ':vel_r (vel_r msg))
))
