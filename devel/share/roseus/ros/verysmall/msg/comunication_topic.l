;; Auto-generated. Do not edit!


(when (boundp 'verysmall::comunication_topic)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow 'comunication_topic (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::COMUNICATION_TOPIC")
  (make-package "VERYSMALL::COMUNICATION_TOPIC"))

(in-package "ROS")
;;//! \htmlinclude comunication_topic.msg.html


(defclass verysmall::comunication_topic
  :super ros::object
  :slots (_robot_id _robots_speed ))

(defmethod verysmall::comunication_topic
  (:init
   (&key
    ((:robot_id __robot_id) (make-array 10 :initial-element 0 :element-type :char))
    ((:robots_speed __robots_speed) (let (r) (dotimes (i 10) (push (instance verysmall::motor_speed :init) r)) r))
    )
   (send-super :init)
   (setq _robot_id __robot_id)
   (setq _robots_speed __robots_speed)
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:robots_speed
   (&rest __robots_speed)
   (if (keywordp (car __robots_speed))
       (send* _robots_speed __robots_speed)
     (progn
       (if __robots_speed (setq _robots_speed (car __robots_speed)))
       _robots_speed)))
  (:serialization-length
   ()
   (+
    ;; uint8[10] _robot_id
    (* 1    10)
    ;; verysmall/motor_speed[10] _robots_speed
    (apply #'+ (send-all _robots_speed :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[10] _robot_id
     (princ _robot_id s)
     ;; verysmall/motor_speed[10] _robots_speed
     (dolist (elem _robots_speed)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[10] _robot_id
   (setq _robot_id (make-array 10 :element-type :char))
   (replace _robot_id buf :start2 ptr-) (incf ptr- 10)
   ;; verysmall/motor_speed[10] _robots_speed
   (dotimes (i 10)
     (send (elt _robots_speed i) :deserialize buf ptr-) (incf ptr- (send (elt _robots_speed i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get verysmall::comunication_topic :md5sum-) "bcfc10b8d53657d2a18ba160b07bee7b")
(setf (get verysmall::comunication_topic :datatype-) "verysmall/comunication_topic")
(setf (get verysmall::comunication_topic :definition-)
      "uint8[10]          robot_id
motor_speed[10]    robots_speed
================================================================================
MSG: verysmall/motor_speed
int32            vel_l
int32            vel_r
")



(provide :verysmall/comunication_topic "bcfc10b8d53657d2a18ba160b07bee7b")


