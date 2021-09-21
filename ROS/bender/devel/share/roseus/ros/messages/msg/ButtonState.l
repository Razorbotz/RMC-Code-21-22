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
  :slots (_number _state ))

(defmethod messages::ButtonState
  (:init
   (&key
    ((:number __number) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _number (round __number))
   (setq _state (round __state))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; int32 _number
    4
    ;; int32 _state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _number
       (write-long _number s)
     ;; int32 _state
       (write-long _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _number
     (setq _number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::ButtonState :md5sum-) "09160d212a568b0a18a64adb7eb7b9d1")
(setf (get messages::ButtonState :datatype-) "messages/ButtonState")
(setf (get messages::ButtonState :definition-)
      "int32 number 
int32 state

")



(provide :messages/ButtonState "09160d212a568b0a18a64adb7eb7b9d1")


