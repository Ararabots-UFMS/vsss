
(cl:in-package :asdf)

(defsystem "verysmall-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "5_robot_pos" :depends-on ("_package_5_robot_pos"))
    (:file "_package_5_robot_pos" :depends-on ("_package"))
    (:file "5_robot_vector" :depends-on ("_package_5_robot_vector"))
    (:file "_package_5_robot_vector" :depends-on ("_package"))
    (:file "comunication_topic" :depends-on ("_package_comunication_topic"))
    (:file "_package_comunication_topic" :depends-on ("_package"))
    (:file "game_topic" :depends-on ("_package_game_topic"))
    (:file "_package_game_topic" :depends-on ("_package"))
    (:file "motor_speed" :depends-on ("_package_motor_speed"))
    (:file "_package_motor_speed" :depends-on ("_package"))
    (:file "things_position" :depends-on ("_package_things_position"))
    (:file "_package_things_position" :depends-on ("_package"))
  ))