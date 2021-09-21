;; Auto-generated. Do not edit!


(when (boundp 'messages::JoystickCommand)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'JoystickCommand (find-package "MESSAGES")))
(unless (find-package "MESSAGES::JOYSTICKCOMMAND")
  (make-package "MESSAGES::JOYSTICKCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude JoystickCommand.msg.html


(defclass messages::JoystickCommand
  :super ros::object
  :slots (_number _element _value ))

(defmethod messages::JoystickCommand
  (:init
   (&key
    ((:number __number) 0)
    ((:element __element) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _number (round __number))
   (setq _element (round __element))
   (setq _value (float __value))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:element
   (&optional __element)
   (if __element (setq _element __element)) _element)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int8 _number
    1
    ;; int8 _element
    1
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _number
       (write-byte _number s)
     ;; int8 _element
       (write-byte _element s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _number
     (setq _number (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _number 127) (setq _number (- _number 256)))
   ;; int8 _element
     (setq _element (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _element 127) (setq _element (- _element 256)))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::JoystickCommand :md5sum-) "90a76c8abd6ced46c905e85492d19a7f")
(setf (get messages::JoystickCommand :datatype-) "messages/JoystickCommand")
(setf (get messages::JoystickCommand :definition-)
      "int8 number
int8 element
float32 value

")



(provide :messages/JoystickCommand "90a76c8abd6ced46c905e85492d19a7f")


