; Auto-generated. Do not edit!


(cl:in-package verysmall-msg)


;//! \htmlinclude comunication_topic.msg.html

(cl:defclass <comunication_topic> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 10 :element-type 'cl:fixnum :initial-element 0))
   (robots_speed
    :reader robots_speed
    :initarg :robots_speed
    :type (cl:vector verysmall-msg:motor_speed)
   :initform (cl:make-array 10 :element-type 'verysmall-msg:motor_speed :initial-element (cl:make-instance 'verysmall-msg:motor_speed))))
)

(cl:defclass comunication_topic (<comunication_topic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <comunication_topic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'comunication_topic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name verysmall-msg:<comunication_topic> is deprecated: use verysmall-msg:comunication_topic instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <comunication_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robot_id-val is deprecated.  Use verysmall-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'robots_speed-val :lambda-list '(m))
(cl:defmethod robots_speed-val ((m <comunication_topic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader verysmall-msg:robots_speed-val is deprecated.  Use verysmall-msg:robots_speed instead.")
  (robots_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <comunication_topic>) ostream)
  "Serializes a message object of type '<comunication_topic>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'robot_id))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robots_speed))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <comunication_topic>) istream)
  "Deserializes a message object of type '<comunication_topic>"
  (cl:setf (cl:slot-value msg 'robot_id) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'robot_id)))
    (cl:dotimes (i 10)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'robots_speed) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'robots_speed)))
    (cl:dotimes (i 10)
    (cl:setf (cl:aref vals i) (cl:make-instance 'verysmall-msg:motor_speed))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<comunication_topic>)))
  "Returns string type for a message object of type '<comunication_topic>"
  "verysmall/comunication_topic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'comunication_topic)))
  "Returns string type for a message object of type 'comunication_topic"
  "verysmall/comunication_topic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<comunication_topic>)))
  "Returns md5sum for a message object of type '<comunication_topic>"
  "bcfc10b8d53657d2a18ba160b07bee7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'comunication_topic)))
  "Returns md5sum for a message object of type 'comunication_topic"
  "bcfc10b8d53657d2a18ba160b07bee7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<comunication_topic>)))
  "Returns full string definition for message of type '<comunication_topic>"
  (cl:format cl:nil "uint8[10]          robot_id~%motor_speed[10]    robots_speed~%================================================================================~%MSG: verysmall/motor_speed~%int32            vel_l~%int32            vel_r~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'comunication_topic)))
  "Returns full string definition for message of type 'comunication_topic"
  (cl:format cl:nil "uint8[10]          robot_id~%motor_speed[10]    robots_speed~%================================================================================~%MSG: verysmall/motor_speed~%int32            vel_l~%int32            vel_r~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <comunication_topic>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robots_speed) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <comunication_topic>))
  "Converts a ROS message object to a list"
  (cl:list 'comunication_topic
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':robots_speed (robots_speed msg))
))
