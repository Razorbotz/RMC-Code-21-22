;; Auto-generated. Do not edit!


(when (boundp 'messages::ButtonState)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'ButtonState (find-package "MESSAGES")))
(unless (find-package "MESSAGES::BUTTONSTATE")
  (make-package "MESSAGES::BUTTONSTATE"))

(in-package "ROS")
;;//! \htmlinclude ButtonState.msg.html


(defclass messages::ButtonState
  :super ros::object
  :slots (_joystick _button _state ))

(defmethod messages::ButtonState
  (:init
   (&key
    ((:joystick __joystick) 0)
    ((:button __button) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _joystick (round __joystick))
   (setq _button (round __button))
   (setq _state (round __state))
   self)
  (:joystick
   (&optional __joystick)
   (if __joystick (setq _joystick __joystick)) _joystick)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _joystick
    1
    ;; uint8 _button
    1
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _joystick
       (write-byte _joystick s)
     ;; uint8 _button
       (write-byte _button s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _joystick
     (setq _joystick (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _button
     (setq _button (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get messages::ButtonState :md5sum-) "d97fed8babddf36e3ba7a8b9236726be")
(setf (get messages::ButtonState :datatype-) "messages/ButtonState")
(setf (get messages::ButtonState :definition-)
      "uint8 joystick
uint8 button
uint8 state

")



(provide :messages/ButtonState "d97fed8babddf36e3ba7a8b9236726be")


