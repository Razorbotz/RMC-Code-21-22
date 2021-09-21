;; Auto-generated. Do not edit!


(when (boundp 'messages::TalonOut)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'TalonOut (find-package "MESSAGES")))
(unless (find-package "MESSAGES::TALONOUT")
  (make-package "MESSAGES::TALONOUT"))

(in-package "ROS")
;;//! \htmlinclude TalonOut.msg.html


(defclass messages::TalonOut
  :super ros::object
  :slots (_current _voltage _temperature _position _velocity ))

(defmethod messages::TalonOut
  (:init
   (&key
    ((:current __current) 0.0)
    ((:voltage __voltage) 0.0)
    ((:temperature __temperature) 0.0)
    ((:position __position) 0)
    ((:velocity __velocity) 0)
    )
   (send-super :init)
   (setq _current (float __current))
   (setq _voltage (float __voltage))
   (setq _temperature (float __temperature))
   (setq _position (round __position))
   (setq _velocity (round __velocity))
   self)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; float32 _current
    4
    ;; float32 _voltage
    4
    ;; float32 _temperature
    4
    ;; int32 _position
    4
    ;; int32 _velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _position
       (write-long _position s)
     ;; int32 _velocity
       (write-long _velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::TalonOut :md5sum-) "1dfab55edab53accf343c309dd48ae24")
(setf (get messages::TalonOut :datatype-) "messages/TalonOut")
(setf (get messages::TalonOut :definition-)
      "float32 current
float32 voltage
float32 temperature
int32 position
int32 velocity

")



(provide :messages/TalonOut "1dfab55edab53accf343c309dd48ae24")


