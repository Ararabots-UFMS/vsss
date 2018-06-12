; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude five_robot_pos.msg.html

(cl:defclass <five_robot_pos> (roslisp-msg-protocol:ros-message)
  ((robot_pos_1
    :reader robot_pos_1
    :initarg :robot_pos_1
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (robot_pos_2
    :reader robot_pos_2
    :initarg :robot_pos_2
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (robot_pos_3
    :reader robot_pos_3
    :initarg :robot_pos_3
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (robot_pos_4
    :reader robot_pos_4
    :initarg :robot_pos_4
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (robot_pos_5
    :reader robot_pos_5
    :initarg :robot_pos_5
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass five_robot_pos (<five_robot_pos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <five_robot_pos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'five_robot_pos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<five_robot_pos> is deprecated: use verysmall-msg:five_robot_pos instead.")))

(cl:ensure-generic-function 'robot_pos_1-val :lambda-list '(m))
(cl:defmethod robot_pos_1-val ((m <five_robot_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_pos_1-val is deprecated.  Use verysmall-msg:robot_pos_1 instead.")
  (robot_pos_1 m))

(cl:ensure-generic-function 'robot_pos_2-val :lambda-list '(m))
(cl:defmethod robot_pos_2-val ((m <five_robot_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_pos_2-val is deprecated.  Use verysmall-msg:robot_pos_2 instead.")
  (robot_pos_2 m))

(cl:ensure-generic-function 'robot_pos_3-val :lambda-list '(m))
(cl:defmethod robot_pos_3-val ((m <five_robot_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_pos_3-val is deprecated.  Use verysmall-msg:robot_pos_3 instead.")
  (robot_pos_3 m))

(cl:ensure-generic-function 'robot_pos_4-val :lambda-list '(m))
(cl:defmethod robot_pos_4-val ((m <five_robot_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_pos_4-val is deprecated.  Use verysmall-msg:robot_pos_4 instead.")
  (robot_pos_4 m))

(cl:ensure-generic-function 'robot_pos_5-val :lambda-list '(m))
(cl:defmethod robot_pos_5-val ((m <five_robot_pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_pos_5-val is deprecated.  Use verysmall-msg:robot_pos_5 instead.")
  (robot_pos_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <five_robot_pos>) ostream)
  "Serializes a message object of type '<five_robot_pos>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_pos_1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_pos_2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_pos_3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_pos_4))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_pos_5))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <five_robot_pos>) istream)
  "Deserializes a message object of type '<five_robot_pos>"
  (cl:setf (cl:slot-value msg 'robot_pos_1) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'robot_pos_1)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'robot_pos_2) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'robot_pos_2)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'robot_pos_3) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'robot_pos_3)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'robot_pos_4) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'robot_pos_4)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'robot_pos_5) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'robot_pos_5)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<five_robot_pos>)))
  "Returns string type for a message object of type '<five_robot_pos>"
  "verysmall/five_robot_pos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'five_robot_pos)))
  "Returns string type for a message object of type 'five_robot_pos"
  "verysmall/five_robot_pos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<five_robot_pos>)))
  "Returns md5sum for a message object of type '<five_robot_pos>"
  "2fb152bde934739c2e0092eef499be2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'five_robot_pos)))
  "Returns md5sum for a message object of type 'five_robot_pos"
  "2fb152bde934739c2e0092eef499be2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<five_robot_pos>)))
  "Returns full string definition for message of type '<five_robot_pos>"
  (cl:format cl:nil "uint32[2]    robot_pos_1~%uint32[2]    robot_pos_2~%uint32[2]    robot_pos_3~%uint32[2]    robot_pos_4~%uint32[2]    robot_pos_5~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'five_robot_pos)))
  "Returns full string definition for message of type 'five_robot_pos"
  (cl:format cl:nil "uint32[2]    robot_pos_1~%uint32[2]    robot_pos_2~%uint32[2]    robot_pos_3~%uint32[2]    robot_pos_4~%uint32[2]    robot_pos_5~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <five_robot_pos>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pos_1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pos_2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pos_3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pos_4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pos_5) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <five_robot_pos>))
  "Converts a ROS message object to a list"
  (cl:list 'five_robot_pos
    (cl:cons ':robot_pos_1 (robot_pos_1 msg))
    (cl:cons ':robot_pos_2 (robot_pos_2 msg))
    (cl:cons ':robot_pos_3 (robot_pos_3 msg))
    (cl:cons ':robot_pos_4 (robot_pos_4 msg))
    (cl:cons ':robot_pos_5 (robot_pos_5 msg))
))
