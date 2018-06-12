; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude game_topic.msg.html

(cl:defclass <game_topic> (roslisp-msg-protocol:ros-message)
  ((robot_roles
    :reader robot_roles
    :initarg :robot_roles
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0))
   (game_state
    :reader game_state
    :initarg :game_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass game_topic (<game_topic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <game_topic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'game_topic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<game_topic> is deprecated: use verysmall-msg:game_topic instead.")))

(cl:ensure-generic-function 'robot_roles-val :lambda-list '(m))
(cl:defmethod robot_roles-val ((m <game_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_roles-val is deprecated.  Use verysmall-msg:robot_roles instead.")
  (robot_roles m))

(cl:ensure-generic-function 'game_state-val :lambda-list '(m))
(cl:defmethod game_state-val ((m <game_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:game_state-val is deprecated.  Use verysmall-msg:game_state instead.")
  (game_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <game_topic>) ostream)
  "Serializes a message object of type '<game_topic>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'robot_roles))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <game_topic>) istream)
  "Deserializes a message object of type '<game_topic>"
  (cl:setf (cl:slot-value msg 'robot_roles) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'robot_roles)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<game_topic>)))
  "Returns string type for a message object of type '<game_topic>"
  "verysmall/game_topic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'game_topic)))
  "Returns string type for a message object of type 'game_topic"
  "verysmall/game_topic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<game_topic>)))
  "Returns md5sum for a message object of type '<game_topic>"
  "609f49595bdc94fe57d847d3ac10e1c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'game_topic)))
  "Returns md5sum for a message object of type 'game_topic"
  "609f49595bdc94fe57d847d3ac10e1c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<game_topic>)))
  "Returns full string definition for message of type '<game_topic>"
  (cl:format cl:nil "uint8[3]     robot_roles~%uint8        game_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'game_topic)))
  "Returns full string definition for message of type 'game_topic"
  (cl:format cl:nil "uint8[3]     robot_roles~%uint8        game_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <game_topic>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_roles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <game_topic>))
  "Converts a ROS message object to a list"
  (cl:list 'game_topic
    (cl:cons ':robot_roles (robot_roles msg))
    (cl:cons ':game_state (game_state msg))
))
