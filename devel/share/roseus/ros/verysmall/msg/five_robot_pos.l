;; Auto-generated. Do not edit!


(when (boundp 'verysmall::five_robot_pos)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow 'five_robot_pos (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::FIVE_ROBOT_POS")
  (make-package "VERYSMALL::FIVE_ROBOT_POS"))

(in-package "ROS")
;;//! \htmlinclude five_robot_pos.msg.html


(defclass verysmall::five_robot_pos
  :super ros::object
  :slots (_robot_pos_1 _robot_pos_2 _robot_pos_3 _robot_pos_4 _robot_pos_5 ))

(defmethod verysmall::five_robot_pos
  (:init
   (&key
    ((:robot_pos_1 __robot_pos_1) (make-array 2 :initial-element 0 :element-type :integer))
    ((:robot_pos_2 __robot_pos_2) (make-array 2 :initial-element 0 :element-type :integer))
    ((:robot_pos_3 __robot_pos_3) (make-array 2 :initial-element 0 :element-type :integer))
    ((:robot_pos_4 __robot_pos_4) (make-array 2 :initial-element 0 :element-type :integer))
    ((:robot_pos_5 __robot_pos_5) (make-array 2 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _robot_pos_1 __robot_pos_1)
   (setq _robot_pos_2 __robot_pos_2)
   (setq _robot_pos_3 __robot_pos_3)
   (setq _robot_pos_4 __robot_pos_4)
   (setq _robot_pos_5 __robot_pos_5)
   self)
  (:robot_pos_1
   (&optional __robot_pos_1)
   (if __robot_pos_1 (setq _robot_pos_1 __robot_pos_1)) _robot_pos_1)
  (:robot_pos_2
   (&optional __robot_pos_2)
   (if __robot_pos_2 (setq _robot_pos_2 __robot_pos_2)) _robot_pos_2)
  (:robot_pos_3
   (&optional __robot_pos_3)
   (if __robot_pos_3 (setq _robot_pos_3 __robot_pos_3)) _robot_pos_3)
  (:robot_pos_4
   (&optional __robot_pos_4)
   (if __robot_pos_4 (setq _robot_pos_4 __robot_pos_4)) _robot_pos_4)
  (:robot_pos_5
   (&optional __robot_pos_5)
   (if __robot_pos_5 (setq _robot_pos_5 __robot_pos_5)) _robot_pos_5)
  (:serialization-length
   ()
   (+
    ;; uint32[2] _robot_pos_1
    (* 4    2)
    ;; uint32[2] _robot_pos_2
    (* 4    2)
    ;; uint32[2] _robot_pos_3
    (* 4    2)
    ;; uint32[2] _robot_pos_4
    (* 4    2)
    ;; uint32[2] _robot_pos_5
    (* 4    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[2] _robot_pos_1
     (dotimes (i 2)
       (write-long (elt _robot_pos_1 i) s)
       )
     ;; uint32[2] _robot_pos_2
     (dotimes (i 2)
       (write-long (elt _robot_pos_2 i) s)
       )
     ;; uint32[2] _robot_pos_3
     (dotimes (i 2)
       (write-long (elt _robot_pos_3 i) s)
       )
     ;; uint32[2] _robot_pos_4
     (dotimes (i 2)
       (write-long (elt _robot_pos_4 i) s)
       )
     ;; uint32[2] _robot_pos_5
     (dotimes (i 2)
       (write-long (elt _robot_pos_5 i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[2] _robot_pos_1
   (dotimes (i (length _robot_pos_1))
     (setf (elt _robot_pos_1 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; uint32[2] _robot_pos_2
   (dotimes (i (length _robot_pos_2))
     (setf (elt _robot_pos_2 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; uint32[2] _robot_pos_3
   (dotimes (i (length _robot_pos_3))
     (setf (elt _robot_pos_3 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; uint32[2] _robot_pos_4
   (dotimes (i (length _robot_pos_4))
     (setf (elt _robot_pos_4 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; uint32[2] _robot_pos_5
   (dotimes (i (length _robot_pos_5))
     (setf (elt _robot_pos_5 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get verysmall::five_robot_pos :md5sum-) "2fb152bde934739c2e0092eef499be2d")
(setf (get verysmall::five_robot_pos :datatype-) "verysmall/five_robot_pos")
(setf (get verysmall::five_robot_pos :definition-)
      "uint32[2]    robot_pos_1
uint32[2]    robot_pos_2
uint32[2]    robot_pos_3
uint32[2]    robot_pos_4
uint32[2]    robot_pos_5
")



(provide :verysmall/five_robot_pos "2fb152bde934739c2e0092eef499be2d")


