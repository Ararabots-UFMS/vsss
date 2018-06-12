; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude things_position.msg.html

(cl:defclass <things_position> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ball_pos
    :reader ball_pos
    :initarg :ball_pos
    :type (cl:vector cl:integer)
   :initform (cl:make-array 2 :element-type 'cl:integer :initial-element 0))
   (ball_vector
    :reader ball_vector
    :initarg :ball_vector
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (team_pos
    :reader team_pos
    :initarg :team_pos
    :type verysmall-msg:5_robot_pos
    :initform (cl:make-instance 'verysmall-msg:5_robot_pos))
   (team_vector
    :reader team_vector
    :initarg :team_vector
    :type verysmall-msg:5_robot_vector
    :initform (cl:make-instance 'verysmall-msg:5_robot_vector))
   (enemies_pos
    :reader enemies_pos
    :initarg :enemies_pos
    :type verysmall-msg:5_robot_pos
    :initform (cl:make-instance 'verysmall-msg:5_robot_pos))
   (enemies_vector
    :reader enemies_vector
    :initarg :enemies_vector
    :type verysmall-msg:5_robot_vector
    :initform (cl:make-instance 'verysmall-msg:5_robot_vector))
   (robot_speed
    :reader robot_speed
    :initarg :robot_speed
    :type (cl:vector cl:integer)
   :initform (cl:make-array 10 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass things_position (<things_position>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <things_position>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'things_position)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<things_position> is deprecated: use verysmall-msg:things_position instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:header-val is deprecated.  Use verysmall-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ball_pos-val :lambda-list '(m))
(cl:defmethod ball_pos-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:ball_pos-val is deprecated.  Use verysmall-msg:ball_pos instead.")
  (ball_pos m))

(cl:ensure-generic-function 'ball_vector-val :lambda-list '(m))
(cl:defmethod ball_vector-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:ball_vector-val is deprecated.  Use verysmall-msg:ball_vector instead.")
  (ball_vector m))

(cl:ensure-generic-function 'team_pos-val :lambda-list '(m))
(cl:defmethod team_pos-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:team_pos-val is deprecated.  Use verysmall-msg:team_pos instead.")
  (team_pos m))

(cl:ensure-generic-function 'team_vector-val :lambda-list '(m))
(cl:defmethod team_vector-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:team_vector-val is deprecated.  Use verysmall-msg:team_vector instead.")
  (team_vector m))

(cl:ensure-generic-function 'enemies_pos-val :lambda-list '(m))
(cl:defmethod enemies_pos-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:enemies_pos-val is deprecated.  Use verysmall-msg:enemies_pos instead.")
  (enemies_pos m))

(cl:ensure-generic-function 'enemies_vector-val :lambda-list '(m))
(cl:defmethod enemies_vector-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:enemies_vector-val is deprecated.  Use verysmall-msg:enemies_vector instead.")
  (enemies_vector m))

(cl:ensure-generic-function 'robot_speed-val :lambda-list '(m))
(cl:defmethod robot_speed-val ((m <things_position>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_speed-val is deprecated.  Use verysmall-msg:robot_speed instead.")
  (robot_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <things_position>) ostream)
  "Serializes a message object of type '<things_position>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'ball_pos))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'ball_vector))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'team_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'team_vector) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'enemies_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'enemies_vector) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'robot_speed))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <things_position>) istream)
  "Deserializes a message object of type '<things_position>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'ball_pos) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'ball_pos)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'ball_vector) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'ball_vector)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'team_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'team_vector) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'enemies_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'enemies_vector) istream)
  (cl:setf (cl:slot-value msg 'robot_speed) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'robot_speed)))
    (cl:dotimes (i 10)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<things_position>)))
  "Returns string type for a message object of type '<things_position>"
  "verysmall/things_position")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'things_position)))
  "Returns string type for a message object of type 'things_position"
  "verysmall/things_position")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<things_position>)))
  "Returns md5sum for a message object of type '<things_position>"
  "f1165e4b4a4a1a0185744f35adc81017")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'things_position)))
  "Returns md5sum for a message object of type 'things_position"
  "f1165e4b4a4a1a0185744f35adc81017")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<things_position>)))
  "Returns full string definition for message of type '<things_position>"
  (cl:format cl:nil "Header                      header~%uint32[2]		 	        ball_pos~%float64[2]        			ball_vector~%verysmall/5_robot_pos       team_pos~%verysmall/5_robot_vector    team_vector~%verysmall/5_robot_pos       enemies_pos~%verysmall/5_robot_vector    enemies_vector~%uint32[10]		 	        robot_speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: verysmall/5_robot_pos~%uint32[2]    robot_pos_1~%uint32[2]    robot_pos_2~%uint32[2]    robot_pos_3~%uint32[2]    robot_pos_4~%uint32[2]    robot_pos_5~%================================================================================~%MSG: verysmall/5_robot_vector~%float64   robot_angle_vector_1~%float64   robot_angle_vector_2~%float64   robot_angle_vector_3~%float64   robot_angle_vector_4~%float64   robot_angle_vector_5~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'things_position)))
  "Returns full string definition for message of type 'things_position"
  (cl:format cl:nil "Header                      header~%uint32[2]		 	        ball_pos~%float64[2]        			ball_vector~%verysmall/5_robot_pos       team_pos~%verysmall/5_robot_vector    team_vector~%verysmall/5_robot_pos       enemies_pos~%verysmall/5_robot_vector    enemies_vector~%uint32[10]		 	        robot_speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: verysmall/5_robot_pos~%uint32[2]    robot_pos_1~%uint32[2]    robot_pos_2~%uint32[2]    robot_pos_3~%uint32[2]    robot_pos_4~%uint32[2]    robot_pos_5~%================================================================================~%MSG: verysmall/5_robot_vector~%float64   robot_angle_vector_1~%float64   robot_angle_vector_2~%float64   robot_angle_vector_3~%float64   robot_angle_vector_4~%float64   robot_angle_vector_5~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <things_position>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ball_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ball_vector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'team_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'team_vector))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'enemies_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'enemies_vector))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <things_position>))
  "Converts a ROS message object to a list"
  (cl:list 'things_position
    (cl:cons ':header (header msg))
    (cl:cons ':ball_pos (ball_pos msg))
    (cl:cons ':ball_vector (ball_vector msg))
    (cl:cons ':team_pos (team_pos msg))
    (cl:cons ':team_vector (team_vector msg))
    (cl:cons ':enemies_pos (enemies_pos msg))
    (cl:cons ':enemies_vector (enemies_vector msg))
    (cl:cons ':robot_speed (robot_speed msg))
))
