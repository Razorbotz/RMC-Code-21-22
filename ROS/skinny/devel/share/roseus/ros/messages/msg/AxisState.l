;; Auto-generated. Do not edit!


(when (boundp 'messages::AxisState)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'AxisState (find-package "MESSAGES")))
(unless (find-package "MESSAGES::AXISSTATE")
  (make-package "MESSAGES::AXISSTATE"))

(in-package "ROS")
;;//! \htmlinclude AxisState.msg.html


(defclass messages::AxisState
  :super ros::object
  :slots (_joystick _axis _state ))

(defmethod messages::AxisState
  (:init
   (&key
    ((:joystick __joystick) 0)
    ((:axis __axis) 0)
    ((:state __state) 0.0)
    )
   (send-super :init)
   (setq _joystick (round __joystick))
   (setq _axis (round __axis))
   (setq _state (float __state))
   self)
  (:joystick
   (&optional __joystick)
   (if __joystick (setq _joystick __joystick)) _joystick)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _joystick
    1
    ;; uint8 _axis
    1
    ;; float32 _state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _joystick
       (write-byte _joystick s)
     ;; uint8 _axis
       (write-byte _axis s)
     ;; float32 _state
       (sys::poke _state (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _joystick
     (setq _joystick (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _axis
     (setq _axis (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _state
     (setq _state (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::AxisState :md5sum-) "66b25cf96600974c62d570e9935b51ff")
(setf (get messages::AxisState :datatype-) "messages/AxisState")
(setf (get messages::AxisState :definition-)
      "uint8 joystick
uint8 axis
float32 state
")



(provide :messages/AxisState "66b25cf96600974c62d570e9935b51ff")


