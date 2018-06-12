; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude 5_robot_vector.msg.html

(cl:defclass <5_robot_vector> (roslisp-msg-protocol:ros-message)
  ((robot_angle_vector_1
    :reader robot_angle_vector_1
    :initarg :robot_angle_vector_1
    :type cl:float
    :initform 0.0)
   (robot_angle_vector_2
    :reader robot_angle_vector_2
    :initarg :robot_angle_vector_2
    :type cl:float
    :initform 0.0)
   (robot_angle_vector_3
    :reader robot_angle_vector_3
    :initarg :robot_angle_vector_3
    :type cl:float
    :initform 0.0)
   (robot_angle_vector_4
    :reader robot_angle_vector_4
    :initarg :robot_angle_vector_4
    :type cl:float
    :initform 0.0)
   (robot_angle_vector_5
    :reader robot_angle_vector_5
    :initarg :robot_angle_vector_5
    :type cl:float
    :initform 0.0))
)

(cl:defclass 5_robot_vector (<5_robot_vector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <5_robot_vector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m '5_robot_vector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<5_robot_vector> is deprecated: use verysmall-msg:5_robot_vector instead.")))

(cl:ensure-generic-function 'robot_angle_vector_1-val :lambda-list '(m))
(cl:defmethod robot_angle_vector_1-val ((m <5_robot_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_angle_vector_1-val is deprecated.  Use verysmall-msg:robot_angle_vector_1 instead.")
  (robot_angle_vector_1 m))

(cl:ensure-generic-function 'robot_angle_vector_2-val :lambda-list '(m))
(cl:defmethod robot_angle_vector_2-val ((m <5_robot_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_angle_vector_2-val is deprecated.  Use verysmall-msg:robot_angle_vector_2 instead.")
  (robot_angle_vector_2 m))

(cl:ensure-generic-function 'robot_angle_vector_3-val :lambda-list '(m))
(cl:defmethod robot_angle_vector_3-val ((m <5_robot_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_angle_vector_3-val is deprecated.  Use verysmall-msg:robot_angle_vector_3 instead.")
  (robot_angle_vector_3 m))

(cl:ensure-generic-function 'robot_angle_vector_4-val :lambda-list '(m))
(cl:defmethod robot_angle_vector_4-val ((m <5_robot_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_angle_vector_4-val is deprecated.  Use verysmall-msg:robot_angle_vector_4 instead.")
  (robot_angle_vector_4 m))

(cl:ensure-generic-function 'robot_angle_vector_5-val :lambda-list '(m))
(cl:defmethod robot_angle_vector_5-val ((m <5_robot_vector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_angle_vector_5-val is deprecated.  Use verysmall-msg:robot_angle_vector_5 instead.")
  (robot_angle_vector_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <5_robot_vector>) ostream)
  "Serializes a message object of type '<5_robot_vector>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle_vector_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle_vector_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle_vector_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle_vector_4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_angle_vector_5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <5_robot_vector>) istream)
  "Deserializes a message object of type '<5_robot_vector>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle_vector_1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle_vector_2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle_vector_3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle_vector_4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_angle_vector_5) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<5_robot_vector>)))
  "Returns string type for a message object of type '<5_robot_vector>"
  "verysmall/5_robot_vector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '5_robot_vector)))
  "Returns string type for a message object of type '5_robot_vector"
  "verysmall/5_robot_vector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<5_robot_vector>)))
  "Returns md5sum for a message object of type '<5_robot_vector>"
  "46d2ecd06a68bed7c3b95711f5808fee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '5_robot_vector)))
  "Returns md5sum for a message object of type '5_robot_vector"
  "46d2ecd06a68bed7c3b95711f5808fee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<5_robot_vector>)))
  "Returns full string definition for message of type '<5_robot_vector>"
  (cl:format cl:nil "float64   robot_angle_vector_1~%float64   robot_angle_vector_2~%float64   robot_angle_vector_3~%float64   robot_angle_vector_4~%float64   robot_angle_vector_5~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '5_robot_vector)))
  "Returns full string definition for message of type '5_robot_vector"
  (cl:format cl:nil "float64   robot_angle_vector_1~%float64   robot_angle_vector_2~%float64   robot_angle_vector_3~%float64   robot_angle_vector_4~%float64   robot_angle_vector_5~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <5_robot_vector>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <5_robot_vector>))
  "Converts a ROS message object to a list"
  (cl:list '5_robot_vector
    (cl:cons ':robot_angle_vector_1 (robot_angle_vector_1 msg))
    (cl:cons ':robot_angle_vector_2 (robot_angle_vector_2 msg))
    (cl:cons ':robot_angle_vector_3 (robot_angle_vector_3 msg))
    (cl:cons ':robot_angle_vector_4 (robot_angle_vector_4 msg))
    (cl:cons ':robot_angle_vector_5 (robot_angle_vector_5 msg))
))
