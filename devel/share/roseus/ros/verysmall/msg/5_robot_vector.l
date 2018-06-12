;; Auto-generated. Do not edit!


(when (boundp 'verysmall::5_robot_vector)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow '5_robot_vector (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::5_ROBOT_VECTOR")
  (make-package "VERYSMALL::5_ROBOT_VECTOR"))

(in-package "ROS")
;;//! \htmlinclude 5_robot_vector.msg.html


(defclass verysmall::5_robot_vector
  :super ros::object
  :slots (_robot_angle_vector_1 _robot_angle_vector_2 _robot_angle_vector_3 _robot_angle_vector_4 _robot_angle_vector_5 ))

(defmethod verysmall::5_robot_vector
  (:init
   (&key
    ((:robot_angle_vector_1 __robot_angle_vector_1) 0.0)
    ((:robot_angle_vector_2 __robot_angle_vector_2) 0.0)
    ((:robot_angle_vector_3 __robot_angle_vector_3) 0.0)
    ((:robot_angle_vector_4 __robot_angle_vector_4) 0.0)
    ((:robot_angle_vector_5 __robot_angle_vector_5) 0.0)
    )
   (send-super :init)
   (setq _robot_angle_vector_1 (float __robot_angle_vector_1))
   (setq _robot_angle_vector_2 (float __robot_angle_vector_2))
   (setq _robot_angle_vector_3 (float __robot_angle_vector_3))
   (setq _robot_angle_vector_4 (float __robot_angle_vector_4))
   (setq _robot_angle_vector_5 (float __robot_angle_vector_5))
   self)
  (:robot_angle_vector_1
   (&optional __robot_angle_vector_1)
   (if __robot_angle_vector_1 (setq _robot_angle_vector_1 __robot_angle_vector_1)) _robot_angle_vector_1)
  (:robot_angle_vector_2
   (&optional __robot_angle_vector_2)
   (if __robot_angle_vector_2 (setq _robot_angle_vector_2 __robot_angle_vector_2)) _robot_angle_vector_2)
  (:robot_angle_vector_3
   (&optional __robot_angle_vector_3)
   (if __robot_angle_vector_3 (setq _robot_angle_vector_3 __robot_angle_vector_3)) _robot_angle_vector_3)
  (:robot_angle_vector_4
   (&optional __robot_angle_vector_4)
   (if __robot_angle_vector_4 (setq _robot_angle_vector_4 __robot_angle_vector_4)) _robot_angle_vector_4)
  (:robot_angle_vector_5
   (&optional __robot_angle_vector_5)
   (if __robot_angle_vector_5 (setq _robot_angle_vector_5 __robot_angle_vector_5)) _robot_angle_vector_5)
  (:serialization-length
   ()
   (+
    ;; float64 _robot_angle_vector_1
    8
    ;; float64 _robot_angle_vector_2
    8
    ;; float64 _robot_angle_vector_3
    8
    ;; float64 _robot_angle_vector_4
    8
    ;; float64 _robot_angle_vector_5
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _robot_angle_vector_1
       (sys::poke _robot_angle_vector_1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _robot_angle_vector_2
       (sys::poke _robot_angle_vector_2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _robot_angle_vector_3
       (sys::poke _robot_angle_vector_3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _robot_angle_vector_4
       (sys::poke _robot_angle_vector_4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _robot_angle_vector_5
       (sys::poke _robot_angle_vector_5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _robot_angle_vector_1
     (setq _robot_angle_vector_1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _robot_angle_vector_2
     (setq _robot_angle_vector_2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _robot_angle_vector_3
     (setq _robot_angle_vector_3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _robot_angle_vector_4
     (setq _robot_angle_vector_4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _robot_angle_vector_5
     (setq _robot_angle_vector_5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get verysmall::5_robot_vector :md5sum-) "46d2ecd06a68bed7c3b95711f5808fee")
(setf (get verysmall::5_robot_vector :datatype-) "verysmall/5_robot_vector")
(setf (get verysmall::5_robot_vector :definition-)
      "float64   robot_angle_vector_1
float64   robot_angle_vector_2
float64   robot_angle_vector_3
float64   robot_angle_vector_4
float64   robot_angle_vector_5
")



(provide :verysmall/5_robot_vector "46d2ecd06a68bed7c3b95711f5808fee")


