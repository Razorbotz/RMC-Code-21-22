;; Auto-generated. Do not edit!


(when (boundp 'messages::HatState)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'HatState (find-package "MESSAGES")))
(unless (find-package "MESSAGES::HATSTATE")
  (make-package "MESSAGES::HATSTATE"))

(in-package "ROS")
;;//! \htmlinclude HatState.msg.html


(defclass messages::HatState
  :super ros::object
  :slots (_joystick _hat _state ))

(defmethod messages::HatState
  (:init
   (&key
    ((:joystick __joystick) 0)
    ((:hat __hat) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _joystick (round __joystick))
   (setq _hat (round __hat))
   (setq _state (round __state))
   self)
  (:joystick
   (&optional __joystick)
   (if __joystick (setq _joystick __joystick)) _joystick)
  (:hat
   (&optional __hat)
   (if __hat (setq _hat __hat)) _hat)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _joystick
    1
    ;; uint8 _hat
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
     ;; uint8 _hat
       (write-byte _hat s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _joystick
     (setq _joystick (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hat
     (setq _hat (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get messages::HatState :md5sum-) "b0f14c98250145be7590c9d7a97bb21b")
(setf (get messages::HatState :datatype-) "messages/HatState")
(setf (get messages::HatState :definition-)
      "uint8 joystick
uint8 hat
uint8 state

")



(provide :messages/HatState "b0f14c98250145be7590c9d7a97bb21b")


