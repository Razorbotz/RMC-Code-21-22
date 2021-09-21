;; Auto-generated. Do not edit!


(when (boundp 'messages::VictorOut)
  (if (not (find-package "MESSAGES"))
    (make-package "MESSAGES"))
  (shadow 'VictorOut (find-package "MESSAGES")))
(unless (find-package "MESSAGES::VICTOROUT")
  (make-package "MESSAGES::VICTOROUT"))

(in-package "ROS")
;;//! \htmlinclude VictorOut.msg.html


(defclass messages::VictorOut
  :super ros::object
  :slots (_deviceID _busVoltage _outputVoltage _outputPercent ))

(defmethod messages::VictorOut
  (:init
   (&key
    ((:deviceID __deviceID) 0)
    ((:busVoltage __busVoltage) 0.0)
    ((:outputVoltage __outputVoltage) 0.0)
    ((:outputPercent __outputPercent) 0.0)
    )
   (send-super :init)
   (setq _deviceID (round __deviceID))
   (setq _busVoltage (float __busVoltage))
   (setq _outputVoltage (float __outputVoltage))
   (setq _outputPercent (float __outputPercent))
   self)
  (:deviceID
   (&optional __deviceID)
   (if __deviceID (setq _deviceID __deviceID)) _deviceID)
  (:busVoltage
   (&optional __busVoltage)
   (if __busVoltage (setq _busVoltage __busVoltage)) _busVoltage)
  (:outputVoltage
   (&optional __outputVoltage)
   (if __outputVoltage (setq _outputVoltage __outputVoltage)) _outputVoltage)
  (:outputPercent
   (&optional __outputPercent)
   (if __outputPercent (setq _outputPercent __outputPercent)) _outputPercent)
  (:serialization-length
   ()
   (+
    ;; int32 _deviceID
    4
    ;; float32 _busVoltage
    4
    ;; float32 _outputVoltage
    4
    ;; float32 _outputPercent
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _deviceID
       (write-long _deviceID s)
     ;; float32 _busVoltage
       (sys::poke _busVoltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _outputVoltage
       (sys::poke _outputVoltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _outputPercent
       (sys::poke _outputPercent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _deviceID
     (setq _deviceID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _busVoltage
     (setq _busVoltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _outputVoltage
     (setq _outputVoltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _outputPercent
     (setq _outputPercent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get messages::VictorOut :md5sum-) "1308269e2bd460ac3f55ed9f4bcd8d51")
(setf (get messages::VictorOut :datatype-) "messages/VictorOut")
(setf (get messages::VictorOut :definition-)
      "int32 deviceID
float32 busVoltage
float32 outputVoltage
float32 outputPercent

")



(provide :messages/VictorOut "1308269e2bd460ac3f55ed9f4bcd8d51")


