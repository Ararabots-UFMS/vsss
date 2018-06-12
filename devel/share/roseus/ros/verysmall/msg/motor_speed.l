;; Auto-generated. Do not edit!


(when (boundp 'verysmall::motor_speed)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow 'motor_speed (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::MOTOR_SPEED")
  (make-package "VERYSMALL::MOTOR_SPEED"))

(in-package "ROS")
;;//! \htmlinclude motor_speed.msg.html


(defclass verysmall::motor_speed
  :super ros::object
  :slots (_vel_l _vel_r ))

(defmethod verysmall::motor_speed
  (:init
   (&key
    ((:vel_l __vel_l) 0)
    ((:vel_r __vel_r) 0)
    )
   (send-super :init)
   (setq _vel_l (round __vel_l))
   (setq _vel_r (round __vel_r))
   self)
  (:vel_l
   (&optional __vel_l)
   (if __vel_l (setq _vel_l __vel_l)) _vel_l)
  (:vel_r
   (&optional __vel_r)
   (if __vel_r (setq _vel_r __vel_r)) _vel_r)
  (:serialization-length
   ()
   (+
    ;; int32 _vel_l
    4
    ;; int32 _vel_r
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _vel_l
       (write-long _vel_l s)
     ;; int32 _vel_r
       (write-long _vel_r s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _vel_l
     (setq _vel_l (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _vel_r
     (setq _vel_r (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get verysmall::motor_speed :md5sum-) "f2dd34999ee8b444344eaa77fc91500c")
(setf (get verysmall::motor_speed :datatype-) "verysmall/motor_speed")
(setf (get verysmall::motor_speed :definition-)
      "int32            vel_l
int32            vel_r
")



(provide :verysmall/motor_speed "f2dd34999ee8b444344eaa77fc91500c")


