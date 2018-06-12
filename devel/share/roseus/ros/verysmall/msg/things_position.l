;; Auto-generated. Do not edit!


(when (boundp 'verysmall::things_position)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow 'things_position (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::THINGS_POSITION")
  (make-package "VERYSMALL::THINGS_POSITION"))

(in-package "ROS")
;;//! \htmlinclude things_position.msg.html


(defclass verysmall::things_position
  :super ros::object
  :slots (_ball_pos _ball_vector _team_pos _team_vector _enemies_pos _enemies_vector _robot_speed ))

(defmethod verysmall::things_position
  (:init
   (&key
    ((:ball_pos __ball_pos) (make-array 2 :initial-element 0 :element-type :integer))
    ((:ball_vector __ball_vector) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:team_pos __team_pos) (instance verysmall::five_robot_pos :init))
    ((:team_vector __team_vector) (instance verysmall::five_robot_vector :init))
    ((:enemies_pos __enemies_pos) (instance verysmall::five_robot_pos :init))
    ((:enemies_vector __enemies_vector) (instance verysmall::five_robot_vector :init))
    ((:robot_speed __robot_speed) (make-array 10 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _ball_pos __ball_pos)
   (setq _ball_vector __ball_vector)
   (setq _team_pos __team_pos)
   (setq _team_vector __team_vector)
   (setq _enemies_pos __enemies_pos)
   (setq _enemies_vector __enemies_vector)
   (setq _robot_speed __robot_speed)
   self)
  (:ball_pos
   (&optional __ball_pos)
   (if __ball_pos (setq _ball_pos __ball_pos)) _ball_pos)
  (:ball_vector
   (&optional __ball_vector)
   (if __ball_vector (setq _ball_vector __ball_vector)) _ball_vector)
  (:team_pos
   (&rest __team_pos)
   (if (keywordp (car __team_pos))
       (send* _team_pos __team_pos)
     (progn
       (if __team_pos (setq _team_pos (car __team_pos)))
       _team_pos)))
  (:team_vector
   (&rest __team_vector)
   (if (keywordp (car __team_vector))
       (send* _team_vector __team_vector)
     (progn
       (if __team_vector (setq _team_vector (car __team_vector)))
       _team_vector)))
  (:enemies_pos
   (&rest __enemies_pos)
   (if (keywordp (car __enemies_pos))
       (send* _enemies_pos __enemies_pos)
     (progn
       (if __enemies_pos (setq _enemies_pos (car __enemies_pos)))
       _enemies_pos)))
  (:enemies_vector
   (&rest __enemies_vector)
   (if (keywordp (car __enemies_vector))
       (send* _enemies_vector __enemies_vector)
     (progn
       (if __enemies_vector (setq _enemies_vector (car __enemies_vector)))
       _enemies_vector)))
  (:robot_speed
   (&optional __robot_speed)
   (if __robot_speed (setq _robot_speed __robot_speed)) _robot_speed)
  (:serialization-length
   ()
   (+
    ;; uint32[2] _ball_pos
    (* 4    2)
    ;; float64[2] _ball_vector
    (* 8    2)
    ;; verysmall/five_robot_pos _team_pos
    (send _team_pos :serialization-length)
    ;; verysmall/five_robot_vector _team_vector
    (send _team_vector :serialization-length)
    ;; verysmall/five_robot_pos _enemies_pos
    (send _enemies_pos :serialization-length)
    ;; verysmall/five_robot_vector _enemies_vector
    (send _enemies_vector :serialization-length)
    ;; uint32[10] _robot_speed
    (* 4    10)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[2] _ball_pos
     (dotimes (i 2)
       (write-long (elt _ball_pos i) s)
       )
     ;; float64[2] _ball_vector
     (dotimes (i 2)
       (sys::poke (elt _ball_vector i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; verysmall/five_robot_pos _team_pos
       (send _team_pos :serialize s)
     ;; verysmall/five_robot_vector _team_vector
       (send _team_vector :serialize s)
     ;; verysmall/five_robot_pos _enemies_pos
       (send _enemies_pos :serialize s)
     ;; verysmall/five_robot_vector _enemies_vector
       (send _enemies_vector :serialize s)
     ;; uint32[10] _robot_speed
     (dotimes (i 10)
       (write-long (elt _robot_speed i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[2] _ball_pos
   (dotimes (i (length _ball_pos))
     (setf (elt _ball_pos i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; float64[2] _ball_vector
   (dotimes (i (length _ball_vector))
     (setf (elt _ball_vector i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; verysmall/five_robot_pos _team_pos
     (send _team_pos :deserialize buf ptr-) (incf ptr- (send _team_pos :serialization-length))
   ;; verysmall/five_robot_vector _team_vector
     (send _team_vector :deserialize buf ptr-) (incf ptr- (send _team_vector :serialization-length))
   ;; verysmall/five_robot_pos _enemies_pos
     (send _enemies_pos :deserialize buf ptr-) (incf ptr- (send _enemies_pos :serialization-length))
   ;; verysmall/five_robot_vector _enemies_vector
     (send _enemies_vector :deserialize buf ptr-) (incf ptr- (send _enemies_vector :serialization-length))
   ;; uint32[10] _robot_speed
   (dotimes (i (length _robot_speed))
     (setf (elt _robot_speed i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get verysmall::things_position :md5sum-) "e53313ec081a9c87ca55fbeaff276969")
(setf (get verysmall::things_position :datatype-) "verysmall/things_position")
(setf (get verysmall::things_position :definition-)
      "uint32[2]		 	        	ball_pos
float64[2]        				ball_vector
verysmall/five_robot_pos       	team_pos
verysmall/five_robot_vector    	team_vector
verysmall/five_robot_pos       	enemies_pos
verysmall/five_robot_vector   	enemies_vector
uint32[10]		 	        	robot_speed

================================================================================
MSG: verysmall/five_robot_pos
uint32[2]    robot_pos_1
uint32[2]    robot_pos_2
uint32[2]    robot_pos_3
uint32[2]    robot_pos_4
uint32[2]    robot_pos_5
================================================================================
MSG: verysmall/five_robot_vector
float64   robot_angle_vector_1
float64   robot_angle_vector_2
float64   robot_angle_vector_3
float64   robot_angle_vector_4
float64   robot_angle_vector_5
")



(provide :verysmall/things_position "e53313ec081a9c87ca55fbeaff276969")


