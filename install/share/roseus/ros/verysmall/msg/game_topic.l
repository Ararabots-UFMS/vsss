;; Auto-generated. Do not edit!


(when (boundp 'verysmall::game_topic)
  (if (not (find-package "VERYSMALL"))
    (make-package "VERYSMALL"))
  (shadow 'game_topic (find-package "VERYSMALL")))
(unless (find-package "VERYSMALL::GAME_TOPIC")
  (make-package "VERYSMALL::GAME_TOPIC"))

(in-package "ROS")
;;//! \htmlinclude game_topic.msg.html


(defclass verysmall::game_topic
  :super ros::object
  :slots (_robot_roles _game_state ))

(defmethod verysmall::game_topic
  (:init
   (&key
    ((:robot_roles __robot_roles) (make-array 3 :initial-element 0 :element-type :char))
    ((:game_state __game_state) 0)
    )
   (send-super :init)
   (setq _robot_roles __robot_roles)
   (setq _game_state (round __game_state))
   self)
  (:robot_roles
   (&optional __robot_roles)
   (if __robot_roles (setq _robot_roles __robot_roles)) _robot_roles)
  (:game_state
   (&optional __game_state)
   (if __game_state (setq _game_state __game_state)) _game_state)
  (:serialization-length
   ()
   (+
    ;; uint8[3] _robot_roles
    (* 1    3)
    ;; uint8 _game_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[3] _robot_roles
     (princ _robot_roles s)
     ;; uint8 _game_state
       (write-byte _game_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[3] _robot_roles
   (setq _robot_roles (make-array 3 :element-type :char))
   (replace _robot_roles buf :start2 ptr-) (incf ptr- 3)
   ;; uint8 _game_state
     (setq _game_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get verysmall::game_topic :md5sum-) "609f49595bdc94fe57d847d3ac10e1c6")
(setf (get verysmall::game_topic :datatype-) "verysmall/game_topic")
(setf (get verysmall::game_topic :definition-)
      "uint8[3]     robot_roles
uint8        game_state

")



(provide :verysmall/game_topic "609f49595bdc94fe57d847d3ac10e1c6")


